# Raycaster Demo
A simple Wolfenstein 3D-style raycaster for Windows. Contained in a single C++ file. Intended as a personal learning tool and interactive demonstration of the rendering techniques a "simple" rendering approach like this requires. It offers toggleable "naive" implementations of various renderer features to show how they distort things.

The renderer operates at a native resolution of 320×240 and scales up to fill the window. The window is freely resizable and supports maximizing to fullscreen.

Written with the help of Claude.

## Building

Just run the release executable. If you'd like to build it yourself, you'll need MSVC. From a Developer Command Prompt:

```
cl /O2 /DUNICODE /D_UNICODE raycaster.cpp /link gdi32.lib user32.lib /subsystem:windows
```

## Controls

| Key | Action |
|-----|--------|
| W / S | Move forward / backward |
| A / D | Turn left / right |
| R | Reset all toggles to defaults |
| Escape | Quit program |

### Rendering toggles

| Key | Effect | Description |
|-----|--------|----------------------|
| X | **Euclidean fisheye**: use raw Euclidean distance instead of perpendicular distance | Without perpendicular distance correction, walls bow outward at the screen edges. The further a ray deviates from the center of the screen, the more its straight-line distance to the wall exceeds the correct perpendicular distance, causing those columns to render taller than they should. |
| V | **Naive ray march**: replace DDA with fixed-increment stepping | Instead of jumping exactly to each grid boundary, the ray advances in small fixed steps. It loses wall face shading, since the march has no concept of which face was entered. Walls bow inward (concave) as a sampling artifact, because rays at oblique angles from the player (i.e. nearest the screen edges) tend to overshoot farther into walls than ones close to the view center. |
| N | **No shading**: disable wall face shading | Walls facing north/south are normally drawn at 67% brightness to simulate a directional light. Without this, all walls render at full white and the scene loses much of its depth. |
| M | **Fixed slice height**: ignore distance when sizing wall slices | Normally each wall slice is drawn taller the closer it is (i.e. in perspective). With a fixed height, all slices are the same size regardless of distance; walls become a uniform band with no depth cue. |
| T / G | **FOV**: increase/decrease field of view | Increases or decreases the field of view. Distortion from other toggles tends to increase markedly at higher FOVs. |
| Y / H | **Column width step**: increase/decrease rays cast per frame | Normally one ray is cast per one-pixel screen column. Increasing the step casts fewer rays and repeats each result across multiple columns, reducing resolution. Steps cycle in increments of two pixels. |
| B | **Unbuffered rendering**: write unbuffered pixels directly to the screen | Normally the frame is rendered into an offscreen buffer and copied to the screen in one go. In unbuffered mode, each pixel is written to the screen as it is computed. If the monitor scans out the frame mid-render, which it will, the result is hideous tearing. |

### Timing toggles

| Key | Effect | What it demonstrates |
|-----|--------|----------------------|
| C | **Per-frame ("raw") movement**: disable delta-time scaling | Normally movement speed is multiplied by elapsed frame time so it remains consistent at any frame rate. With this off, each frame moves a fixed distance regardless of how long it took. Without lag mode enabled this will probably be extremely fast on your system |
| J | **Lag mode**: toggle ~12 fps simulation | Adds an 83ms sleep each frame, throttling the program to roughly 12 fps. With naive ray march enabled, this makes the speed difference visible. With it disabled, delta-time scaling keeps movement consistent despite the lowered frame rate. |

## How it works
The world is a 2D grid of cells that are either empty (traversable) or solid (walls). For each vertical column of the screen, a ray is cast from the player's position into the world. That ray is advanced cell-by-cell until it hits a wall. The distance computed between the player and that wall is used to determine the proportional height of the wall on that column. Closer walls produce taller slices and vice versa.

The player's view is defined by a direction vector. A camera plane vector, perpendicular to the direction vector, controls perpendicular distance correction during rendering and the current field of view. When perpendicular distance correction is enabled, rays are cast between points on the camera plane and corresponding screen columns. The ray length for that column is the sum of the player's direction vector and the corresponding offset on the camera plane. This produces an undistorted view.
