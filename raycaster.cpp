#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <chrono>                // for frame timing

using namespace std;

static const int    SW        = 320;          // framebuffer width/height in pixels (native render resolution)
static const int    SH        = 240;        
static const int    SCALE     = 3;            // initial window scale: window opens at SW*SCALE x SH*SCALE, i.e. 960x720 with default 320/240 values
static const int    WW        = SW * SCALE;   // initial window width/height in pixels
static const int    WH        = SH * SCALE;

static const double MOV_SPEED = 2.5;          // player movement/rotation speeds, in map cells/radians per second respectively
static const double ROT_SPEED = 2.0;
static const double MARGIN    = 0.10;         // how close you can get to a wall before being stopped: your "width"
static const double MOV_STEP_RAW = MOV_SPEED / 60.0; // fixed per-frame movement and rotation steps (when delta-T disabled)
static const double ROT_STEP_RAW = ROT_SPEED / 60.0;
static const double MARCH_STEP = 0.02;  // distance between steps in the naive ray march; if larger than cell size, it can miss walls entirely
static const double MARCH_MAX  = 30.0;  // maximum distance a marching ray will travel before giving up

static bool g_lag = false; // toggles simulated frame drop

// Map.
// The world is a 2D grid of cells. 1 = solid wall, 0 = empty space.
// Row 0 is the top of the map; Y increases downward (screen convention).
static const int MAP_W = 20;   // map height and width in cells.
static const int MAP_H = 20;   
static const int MAP[MAP_H][MAP_W] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1},
    {1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1},
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};

// Feature toggles and respective keys.
static bool g_fisheye    = false; // X: use raw Euclidean raytracing distance instead of perpendicular distance, causing fisheye distortion (as if projecting on a sphere and not a flat plane)
static bool g_rawrate    = false; // C: disable delta-time scaling so movement/rotation speed is tied to raw frame rate
static bool g_nodda      = false; // V: replace DDA raytracing with a naive fixed-increment ray march, creating opposite distortion to toggle X.
static bool g_nobuffer   = false; // B: write pixels directly to the screen instead of to an offscreen buffer, causing tearing
static int  g_colStep    = 1;     // Y/H: cast one ray every N columns (Y incrementing, H decrementing); skipped columns copy their neighbour. Essentially, column width. Default 1.
static bool g_noshading  = false; // N: disable the X/Y side shading on walls
static bool g_fixedslice = false; // M: ignore distance-based wall slice height and render them with fixed height. Essentially, removes proportion from distance.

//Win32 globals.
static HWND      g_hwnd     = nullptr; // handle to the application window
static HDC       g_memDC    = nullptr; // device context for the offscreen backbuffer
static HBITMAP   g_bmp      = nullptr; // the offscreen bitmap that backs g_memDC
static uint32_t *g_bits     = nullptr; // direct pointer to the backbuffer's pixel data
static HDC       g_screenDC = nullptr; // device context for the actual window surface; only valid during rendering
static int       g_dispScale = 3;      // how many screen pixels each framebuffer pixel currently occupies
static int       g_dispOX    = 0;      // horizontal offset in screen pixels to center the image (pillarbox)
static int       g_dispOY    = 0;      // vertical offset in screen pixels to center the image (letterbox)

/* Frame buffer.
The framebuffer is a 2D array of 32-bit pixels (0x00RRGGBB) at native 320x240 resolution.
All rendering writes here first; the result is then scaled and copied to the screen (a "blit") instead of working pixel-by-pixel.
With only black and white, this could be an array of 1-bit pixels (~9 kb instead of 300).
However that would require conversions on every blit and so is avoided for simplicity's sake + the use of simple wall shading.
You could work out a dithering approach for that instead.
*/
static uint32_t fb[SH][SW];

static void clear_fb() {
    memset(fb, 0, sizeof(fb)); // blacks the pixels inside the frame buffer out.
}

static void put_pixel(int x, int y, uint32_t color) {
    if (x < 0 || x >= SW || y < 0 || y >= SH) return; // discard out-of-bounds writes
    if (g_nobuffer) {	//toggled with B: naive pixel-by-pixel mode. Because the screen may be scanned out by the monitor mid-frame, this causes tearing.
        COLORREF cr = RGB((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF); // unpack R, G, B channels from packed 32-bit color
        int px = g_dispOX + x * g_dispScale; // screen X and Y of this pixel's top-left corner, affected by display scale
        int py = g_dispOY + y * g_dispScale; 
        for (int sy = 0; sy < g_dispScale; sy++)     // for the pixel's entire scaled height...
            for (int sx = 0; sx < g_dispScale; sx++) // ...and scaled width,
                SetPixel(g_screenDC, px + sx, py + sy, cr); // draw one physical screen pixel.
    } else {
        fb[y][x] = color; //normal buffered mode: just store pixel into the framebuffer array
    }
}

struct Player {
    double x, y;   // position in map space (each cell is 1.0 x 1.0 units)
    double dx, dy; // direction vector: the direction the player is facing; always length 1
    double cx, cy; // camera plane vector: perpendicular to (dx,dy). Length (cy) encodes field of view.
};

/* Raycaster.
For each vertical column of the screen, cast a ray into the map and draw a wall slice 
whose height is inversely proportional to the ray's distance from the player.
*/ 
static void raycast(const Player &p) {
    int lastDrawStart = 0, lastDrawEnd = 0;   // remember the wall slice and color from the last cast column
    uint32_t lastColor = 0xFFFFFF;
	
    for (int col = 0; col < SW; col++) {	//for every column in the screen width:
        if (g_colStep > 1 && (col % g_colStep) != 0) { // if this column is skipped due to reduced granularity...
            for (int row = lastDrawStart; row < lastDrawEnd; row++)
                put_pixel(col, row, lastColor); // ...copy the previous column's slice instead of casting a new ray
            continue;
        }
        double camX = 2.0 * col / SW - 1.0; // map column index to [-1, 1]: -1 is the left edge of screen, +1 is the right

        double rdx = p.dx + p.cx * camX; // ray direction X: player's forward direction offset by camX along the camera plane
        double rdy = p.dy + p.cy * camX; // ditto, for the Y axis

        double rlen = sqrt(rdx * rdx + rdy * rdy); // length of the ray direction vector (> 1 at screen edges due to camera plane offset)
        double rndx = rdx / rlen;                   // normalized ray direction X and Y (length exactly 1); used by the march
        double rndy = rdy / rlen;

        double dist = 0.0; // distance from player to wall hit by raytrace; default 0
        bool   hit  = false;
        int    side = 0; // 0 = ray hit a vertical wall face (X-axis crossing); 1 = ray hit a horizontal face (Y-axis crossing)

		/* "Naive" ray march.
		Steps along the ray in small increments to see if the map cell is empty or a wall each time.
		The more oblique the rays are to the view center (i.e. the farther towards the screen edges), the more overshoot into wall cells
		before the step ends and the hit is recognized --> overestimation of view-edge wall distances --> concave distortion.
		Also, wall shading does not work here, because the raytrace does not recognize if it hit the X or Y-axis edge of a wall.
		*/
        if (g_nodda) {
            for (double t = MARCH_STEP; t < MARCH_MAX; t += MARCH_STEP) {
                double wx = p.x + rndx * t; // world X position of this step
                double wy = p.y + rndy * t; // world Y position of this step
                int mx = (int)wx, my = (int)wy; // convert to map cell coordinates by truncating
                if (mx >= 0 && mx < MAP_W && my >= 0 && my < MAP_H && MAP[my][mx] == 1) { // if this cell is a wall...
                    dist = t;   // record the Euclidean distance marched to reach it
                    hit  = true;
                    break;
                }
            }
        }
		/* DDA (Digital Differential Analysis): instead of stepping by tiny increments,
		jump exactly to the next grid line crossing on the X or Y axis (one cell per iteration).
		*/ 		
		else {
            int mx = (int)p.x; // map cell X and Y coordinates the player currently occupies
            int my = (int)p.y;

            double ddx = (rdx == 0.0) ? 1e30 : fabs(1.0 / rdx); // distance along ray between successive vertical/horizontal grid line crossings
            double ddy = (rdy == 0.0) ? 1e30 : fabs(1.0 / rdy);

            int stepX, stepY;    // which direction to step in X and Y (+1 or -1)
            double sdx, sdy;     // distance along the ray to the first X and Y grid line crossing

            if (rdx < 0) { stepX = -1; sdx = (p.x - mx) * ddx; }        // ray goes left: first X crossing is to the left of the player
            else          { stepX =  1; sdx = (mx + 1.0 - p.x) * ddx; } // ray goes right: first X crossing is to the right

            if (rdy < 0) { stepY = -1; sdy = (p.y - my) * ddy; }        // ray goes forward: first Y crossing is ahead of the player
            else          { stepY =  1; sdy = (my + 1.0 - p.y) * ddy; } // ray goes backwards: first Y crossing is behind the player

            for (int i = 0; i < 64 && !hit; i++) {  // step through cells until a wall is hit (max 64 steps)
                if (sdx < sdy) { sdx += ddx; mx += stepX; side = 0; } // the next grid crossing is in X: advance in X
                else            { sdy += ddy; my += stepY; side = 1; } // the next grid crossing is in Y: advance in Y

                if (mx >= 0 && mx < MAP_W && my >= 0 && my < MAP_H) // if not out of map bounds:
                    if (MAP[my][mx] == 1) hit = true; // if this map coordinate is a wall, then the raytrace found a wall.
            }

            if (hit)
                dist = (side == 0) ? (sdx - ddx) : (sdy - ddy); // perpendicular distance to the wall face that was hit
        }

        if (!hit) continue; // ray escaped the map without hitting anything; leave this column black

        double drawDist; // will be used to compute slice height
        if (g_fisheye) {
            /* Fisheye mode: use the raw Euclidean (straight-line) distance to the hit point.
            Rays at the screen edges have to travel further than center rays to reach the same wall,
            making edge walls appear closer and their slices taller (convex distortion relative to view center).
			*/
            if (g_nodda) {
                drawDist = dist; // march already gives Euclidean distance
            } else {
                double hitX = p.x + rdx * dist;         // where is the hit point's world X and Y?
                double hitY = p.y + rdy * dist;         
                double ex = hitX - p.x, ey = hitY - p.y; // vector from player to that hit point
                drawDist = sqrt(ex * ex + ey * ey);     // length of that vector = Euclidean distance
            }
        } else {
            /* Default mode. Uses perpendicular distance (i.e. ignores any lateral part of the ray vector.)
			Essentially: walls are drawn according to how far they are from a line perpendicular to and passing through the player.
			Thus a flat wall produces a flat row of equal-height slices, with no distortion.
            */ 
            if (g_nodda) {
                drawDist = fabs((p.dx * rdx + p.dy * rdy) / rlen * dist / rlen); // dot the ray direction onto the player's forward direction to project out the lateral component
            } else {
                drawDist = dist; // DDA already gives perpendicular distance directly
            }
        }

        if (drawDist <= 0.0) drawDist = 0.0001; // prevents division by zero for walls right on top of the player

        int sliceH    = g_fixedslice ? (SH / 2) : (int)(SH / drawDist); // wall slice height in pixels: closer = taller; fixed mode ignores distance
        int drawStart = (SH - sliceH) / 2; //the column to be drawn starts at screen height - slice height / 2...
        int drawEnd   = (SH + sliceH) / 2; //...and ends at the same distance to the bottom
        if (drawStart < 0)  drawStart = 0;  // super close slices: clamp to top and bottom of screen
        if (drawEnd   > SH) drawEnd   = SH;

		//Shades Y-side faces with a lighter color. Requires DDA to be on for side info.
        uint32_t wallColor = (!g_noshading && !g_nodda && side == 1) ? 0xAAAAAA : 0xFFFFFF;
        lastDrawStart = drawStart; lastDrawEnd = drawEnd; lastColor = wallColor; // save for column repetition (if enabled) come the next raycaster call

        for (int row = drawStart; row < drawEnd; row++)
            put_pixel(col, row, wallColor); // draw this column's wall slice from top to bottom; ceiling and floor remain black
    }
}

// Create the offscreen backbuffer: a Win32 DIBSection (Device-Independent Bitmap) at native 320x240 resolution.
static void create_backbuffer(HDC screenDC) {
    BITMAPINFO bmi = {};
    bmi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER); // size of the header struct, required by the API
    bmi.bmiHeader.biWidth       = SW;   // bitmap width in pixels
    bmi.bmiHeader.biHeight      = -SH;  // negative height = top-down pixel order (row 0 at the top)
    bmi.bmiHeader.biPlanes      = 1;    // always 1 for device bitmaps
    bmi.bmiHeader.biBitCount    = 32;   // 32 bits per pixel: 8 bits each for R, G, B, and an unused alpha channel
    bmi.bmiHeader.biCompression = BI_RGB; // uncompressed raw pixels

    g_bmp   = CreateDIBSection(screenDC, &bmi, DIB_RGB_COLORS, (void **)&g_bits, nullptr, 0); // create the bitmap and get a direct pointer to its pixel data
    g_memDC = CreateCompatibleDC(screenDC); // offscreen device context compatible with the screen
    SelectObject(g_memDC, g_bmp);           // attach the bitmap to the device context so the drawing goes into it
}

// Copy the framebuffer to the window, scaling it up to fill the client area while
// maintaining the 320x240 aspect ratio. Black bars fill any leftover space.
static void blit(HDC screenDC) {
    for (int y = 0; y < SH; y++)
        for (int x = 0; x < SW; x++)
            g_bits[y * SW + x] = fb[y][x]; // copy framebuffer into the DIBSection pixel-by-pixel

    RECT cr;
    GetClientRect(g_hwnd, &cr);           // get the current size of the window's client area
    int cw = cr.right, ch = cr.bottom;    // client width and height in pixels
    int scale = min(cw / SW, ch / SH);    // largest integer scale factor that fits both dimensions
    if (scale < 1) scale = 1;             // always scale at least 1x even if window is tiny
    int dw = SW * scale, dh = SH * scale; // destination rectangle dimensions
    int ox = (cw - dw) / 2, oy = (ch - dh) / 2; // offset to center the image in the window
    g_dispScale = scale; g_dispOX = ox; g_dispOY = oy; // store for use by put_pixel in unbuffered mode

    if (ox > 0 || oy > 0) { // if there are bars to fill...
        HBRUSH black = (HBRUSH)GetStockObject(BLACK_BRUSH); // solid black brush, OS-provided
        RECT fill;
        if (oy > 0) { // letterbox bars (top and bottom)
            SetRect(&fill, 0, 0, cw, oy);            FillRect(screenDC, &fill, black);
            SetRect(&fill, 0, oy + dh, cw, ch);      FillRect(screenDC, &fill, black);
        }
        if (ox > 0) { // pillarbox bars (left and right)
            SetRect(&fill, 0, oy, ox, oy + dh);      FillRect(screenDC, &fill, black);
            SetRect(&fill, ox + dw, oy, cw, oy + dh);FillRect(screenDC, &fill, black);
        }
    }

    SetStretchBltMode(screenDC, COLORONCOLOR); // use nearest-neighbour scaling: each destination pixel picks the closest source pixel
    StretchBlt(screenDC, ox, oy, dw, dh, g_memDC, 0, 0, SW, SH, SRCCOPY); // copy and scale the backbuffer to the window
}

// Forward declaration: blit is defined above but WndProc below needs to call it
static void blit(HDC screenDC);

// Window setup.
static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_DESTROY) { PostQuitMessage(0); return 0; }
    if (msg == WM_KEYDOWN) {
        if (wp == 'X') g_fisheye    = !g_fisheye;   // toggle fisheye (Euclidean distance)
        if (wp == 'C') g_rawrate    = !g_rawrate;   // toggle raw frame rate (no delta-time)
        if (wp == 'V') g_nodda      = !g_nodda;     // toggle naive ray march instead of DDA
        if (wp == 'B') { g_nobuffer = !g_nobuffer; if (!g_nobuffer) InvalidateRect(hwnd, nullptr, FALSE); } // toggle unbuffered rendering; force redraw when turning it off
        if (wp == 'N') g_noshading  = !g_noshading; // toggle wall face shading (requires DDA)
        if (wp == 'M') g_fixedslice = !g_fixedslice; // toggle fixed slice height
        if (wp == 'Y') { g_colStep = (g_colStep == 1) ? 2 : min(g_colStep + 2, 20); } // increase column step (coarser rendering)
        if (wp == 'H') { g_colStep = (g_colStep <= 2) ? 1 : max(g_colStep - 2, 2); }  // decrease column step (finer rendering)
        if (wp == 'J') g_lag = !g_lag; // toggle lag mode (simulated 12 fps)
        return 0;
    }
    if (msg == WM_PAINT) { // window needs redrawing (e.g. after being uncovered or resized)
        PAINTSTRUCT ps;
        HDC dc = BeginPaint(hwnd, &ps); 
        blit(dc);                       
        EndPaint(hwnd, &ps);            
        return 0;
    }
    return DefWindowProc(hwnd, msg, wp, lp);
}

// WinMain: Windows equivalent of main() for GUI applications (no console window)
int WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR, int) {
    WNDCLASSEX wc    = {};                    
    wc.cbSize        = sizeof(wc);            
    wc.lpfnWndProc   = WndProc;               
    wc.hInstance     = hInst;                 
    wc.hCursor       = LoadCursor(nullptr, IDC_ARROW); 
    wc.lpszClassName = L"RaycasterClass";     
    RegisterClassEx(&wc);                     

    RECT r = { 0, 0, WW, WH };               // desired client area size
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE); // expand the rect to account for title bar and borders
    g_hwnd = CreateWindowEx(0, L"RaycasterClass", L"Raycaster", // window created and titled
        WS_OVERLAPPEDWINDOW,                  
        CW_USEDEFAULT, CW_USEDEFAULT,         
        r.right - r.left, r.bottom - r.top,   
        nullptr, nullptr, hInst, nullptr);

    HDC screenDC = GetDC(g_hwnd);    
    create_backbuffer(screenDC);     
    ReleaseDC(g_hwnd, screenDC);     

    ShowWindow(g_hwnd, SW_SHOW); 
    UpdateWindow(g_hwnd);        // send an immediate WM_PAINT so the window draws before the game loop starts

    Player p;
    p.x  = 1.5; p.y  = 1.5; // start position
    p.dx = 1.0; p.dy = 0.0; // facing east (positive X direction)
    p.cx = 0.0; p.cy = 0.75; // camera plane perpendicular to direction, length = initial FOV scale (75 degrees)
    double fov = 0.75;        // current FOV scalar; new camera plane length is derived from this each frame

    auto lastTime = chrono::steady_clock::now(); // record the time at the start of the first frame

    // Returns true if the map cell at world position (x, y) is open (not a wall)
    auto cellFree = [](double x, double y) -> bool {
        int cx = (int)x, cy = (int)y;                                                  // truncate to cell coordinates
        return cx >= 0 && cx < MAP_W && cy >= 0 && cy < MAP_H && MAP[cy][cx] == 0;    // bounds check and map lookup
    };

    // Returns true if a player centered at (x, y) would not overlap any wall,
    // by checking whether all four corners of their margin box are in free cells
    auto canOccupy = [&](double x, double y) -> bool {
        return cellFree(x - MARGIN, y - MARGIN) // top-left, top-right, bottom-left, bottom-right corners
            && cellFree(x + MARGIN, y - MARGIN) 
            && cellFree(x - MARGIN, y + MARGIN) 
            && cellFree(x + MARGIN, y + MARGIN);
    };

    // Attempt to move the player to (nx, ny). If the full move is blocked,
    // try sliding along each axis independently so the player slides along walls.
    auto tryMove = [&](double nx, double ny) {
        if      (canOccupy(nx, ny))  { p.x = nx; p.y = ny; } // full move succeeds
        else if (canOccupy(nx, p.y)) { p.x = nx; }            // slide along X only
        else if (canOccupy(p.x, ny)) { p.y = ny; }            // slide along Y only
    };

    // Rotate the player's direction and camera plane vectors by the given angle in radians.
    // Both vectors must be rotated together to keep the camera plane perpendicular to the direction.
    auto rotate = [&](double angle) {
        double cosA = cos(angle), sinA = sin(angle); // precompute sine and cosine of the rotation angle
        double ndx = p.dx * cosA - p.dy * sinA; // standard 2D rotation matrix applied to direction X and Y
        double ndy = p.dx * sinA + p.dy * cosA;
        double ncx = p.cx * cosA - p.cy * sinA; // same rotation applied to camera plane X and Y
        double ncy = p.cx * sinA + p.cy * cosA;
        p.dx = ndx; p.dy = ndy; // update rotated direction and camera plane
        p.cx = ncx; p.cy = ncy;
    };

    MSG msg = {};
    bool running = true;
    while (running) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) { // drain all pending Windows messages without blocking
            if (msg.message == WM_QUIT) { running = false; break; } // quit message breaks the loop
            TranslateMessage(&msg); // translate virtual key codes into character messages
            DispatchMessage(&msg);  // route the message to WndProc
        }
        if (!running) break;

        auto now = chrono::steady_clock::now();
        double dt = chrono::duration<double>(now - lastTime).count(); // elapsed time since last frame in seconds
        lastTime = now;
        if (dt > 0.1) dt = 0.1; // cap dt to prevent huge jumps after stalls (e.g. when dragging the window)

		//Delta-time: scales movement and rotation distances by elapsed time since last frame.
		//Ensures consistency across different framerates. When disabled (frame "raw" mode), you tend to zoom or crawl.
        double moveStep = g_rawrate ? MOV_STEP_RAW : MOV_SPEED * dt; // distance to move this frame: fixed per-frame step in raw mode, delta-time scaled otherwise
        double rotStep  = g_rawrate ? ROT_STEP_RAW : ROT_SPEED * dt; // ditto, angle

        auto key = [](int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; }; // returns true if the given virtual key is currently held down

        if (key(VK_ESCAPE)) { running = false; break; } // exits the program

        if (key('W')) tryMove(p.x + p.dx * moveStep, p.y + p.dy * moveStep); // move forward or backward along direction vector
        if (key('S')) tryMove(p.x - p.dx * moveStep, p.y - p.dy * moveStep);
        if (key('A')) rotate(-rotStep); // turn left (negative rotation) or right (positive rotation)
        if (key('D')) rotate( rotStep);

		//FOV scaling.
        double fovStep = g_rawrate ? 0.15 / 60.0 : 0.15 * dt; // change per frame: fixed per-frame amount in raw mode, delta-time scaled otherwise
        if (key('T')) fov += fovStep; // widen FOV
        if (key('G')) fov -= fovStep; // narrow FOV
        if (key('R')) { fov = 0.75; g_colStep = 1; g_lag = false; g_fisheye = false; g_rawrate = false; g_nodda = false; g_nobuffer = false; g_noshading = false; g_fixedslice = false; } // reset all settings to defaults

        // Reconstruct the camera plane from the current direction and FOV each frame.
        // The perpendicular to a unit vector (dx, dy) is simply (-dy, dx).
        // Scaling by fov sets the camera plane length, which controls field of view width.
        p.cx = -p.dy * fov;
        p.cy =  p.dx * fov;

        g_screenDC = GetDC(g_hwnd); // acquire the window's device context for this frame

        // Recompute display scale and offset each frame so unbuffered put_pixel
        // uses the correct screen coordinates even if the window has been resized.
        {
            RECT cr; GetClientRect(g_hwnd, &cr);
            int cw = cr.right, ch = cr.bottom;
            g_dispScale = min(cw / SW, ch / SH);
            if (g_dispScale < 1) g_dispScale = 1;
            g_dispOX = (cw - SW * g_dispScale) / 2; // center horizontally
            g_dispOY = (ch - SH * g_dispScale) / 2; // center vertically
        }

        if (g_nobuffer) {
            raycast(p); // unbuffered: each put_pixel call goes straight to the screen mid-frame
        } else {
            clear_fb();      // wipe the frame buffer to black before drawing the new frame
            raycast(p);      // render the scene into the frame buffer
            blit(g_screenDC); // copy the completed frame buffer to the screen in one operation
        }

		//Simulated lag.
        if (g_lag) Sleep(83); // ~12 fps: sleep 83ms (1000ms / 12 ≈ 83ms per frame)

        ReleaseDC(g_hwnd, g_screenDC); 
        g_screenDC = nullptr;          // clear the pointer so stray put_pixel calls don't write through a stale handle
    }

    DeleteObject(g_bmp); // free the backbuffer bitmap
    DeleteDC(g_memDC);   
    return 0;
}
