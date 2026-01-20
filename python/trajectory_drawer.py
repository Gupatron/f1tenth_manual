#!/usr/bin/env python3
"""
Trajectory Drawer - Interactive path planning tool for Pure Pursuit
Draw desired robot trajectories and export them for C++ pure pursuit algorithm
"""

import pygame
import sys
import numpy as np
from typing import List, Tuple
import json
from datetime import datetime

# ============================================================================
# CONFIGURATION - Modify these for your lab environment
# ============================================================================

# Physical room dimensions (in meters)
ROOM_WIDTH = 5.0   # meters
ROOM_HEIGHT = 4.0  # meters

# Canvas dimensions (pixels)
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 640

# Drawing settings
DRAW_COLOR = (0, 120, 255)      # Blue
WAYPOINT_COLOR = (255, 0, 0)    # Red for waypoints
BACKGROUND_COLOR = (240, 240, 240)  # Light gray
GRID_COLOR = (200, 200, 200)    # Gray
LINE_WIDTH = 3                  # pixels
POINT_RADIUS = 5                # pixels for waypoints

# Grid settings (optional visual aid)
SHOW_GRID = True
GRID_SPACING_METERS = 0.5  # Grid line every 0.5 meters

# Path processing for Pure Pursuit
MIN_WAYPOINT_SPACING = 0.1  # Minimum distance between waypoints (meters)
RESAMPLE_SPACING = 0.05     # Target spacing for resampled path (meters)

# Export formats
EXPORT_JSON = True
EXPORT_CSV = True
EXPORT_CPP_HEADER = True    # Generate C++ header file

# ============================================================================
# Helper Functions
# ============================================================================

def pixels_to_meters(pixel_coords: Tuple[int, int]) -> Tuple[float, float]:
    """Convert pixel coordinates to physical lab coordinates (meters)"""
    x_pixels, y_pixels = pixel_coords
    
    # Convert to meters
    x_meters = (x_pixels / CANVAS_WIDTH) * ROOM_WIDTH
    # Flip Y axis (pygame Y increases downward, but physical Y increases upward)
    y_meters = ((CANVAS_HEIGHT - y_pixels) / CANVAS_HEIGHT) * ROOM_HEIGHT
    
    return (x_meters, y_meters)

def meters_to_pixels(meter_coords: Tuple[float, float]) -> Tuple[int, int]:
    """Convert physical lab coordinates (meters) to pixel coordinates"""
    x_meters, y_meters = meter_coords
    
    x_pixels = int((x_meters / ROOM_WIDTH) * CANVAS_WIDTH)
    # Flip Y axis
    y_pixels = int(CANVAS_HEIGHT - (y_meters / ROOM_HEIGHT) * CANVAS_HEIGHT)
    
    return (x_pixels, y_pixels)

def smooth_path(points: List[Tuple[int, int]], factor: int) -> List[Tuple[int, int]]:
    """
    Smooth the path by keeping every nth point
    Returns a simplified path with fewer points
    """
    if len(points) <= 2:
        return points
    
    # Keep first point, every nth point, and last point
    smoothed = [points[0]]
    for i in range(factor, len(points), factor):
        smoothed.append(points[i])
    
    # Always include the last point if not already included
    if smoothed[-1] != points[-1]:
        smoothed.append(points[-1])
    
    return smoothed

def resample_path(physical_coords: List[Tuple[float, float]], spacing: float) -> List[Tuple[float, float]]:
    """
    Resample path to have uniform spacing between waypoints
    Critical for pure pursuit algorithm performance
    """
    if len(physical_coords) < 2:
        return physical_coords
    
    resampled = [physical_coords[0]]
    accumulated_dist = 0.0
    
    for i in range(1, len(physical_coords)):
        x1, y1 = physical_coords[i-1]
        x2, y2 = physical_coords[i]
        
        segment_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if segment_length < 1e-6:  # Skip duplicate points
            continue
        
        # Calculate unit direction vector
        dx = (x2 - x1) / segment_length
        dy = (y2 - y1) / segment_length
        
        # Walk along segment adding points at regular intervals
        dist_along_segment = 0.0
        while accumulated_dist + dist_along_segment < segment_length:
            remaining = spacing - accumulated_dist
            dist_along_segment += remaining
            
            if dist_along_segment < segment_length:
                new_x = x1 + dx * dist_along_segment
                new_y = y1 + dy * dist_along_segment
                resampled.append((new_x, new_y))
                accumulated_dist = 0.0
            else:
                accumulated_dist += segment_length - dist_along_segment
                break
        
        if dist_along_segment >= segment_length:
            accumulated_dist = segment_length - (dist_along_segment - spacing + accumulated_dist)
    
    # Always include the last point
    if resampled[-1] != physical_coords[-1]:
        resampled.append(physical_coords[-1])
    
    return resampled

def calculate_curvature(points: List[Tuple[float, float]]) -> List[float]:
    """
    Calculate curvature at each point (useful for speed profiling)
    Returns curvature in 1/meters
    """
    if len(points) < 3:
        return [0.0] * len(points)
    
    curvatures = [0.0]  # First point has zero curvature
    
    for i in range(1, len(points) - 1):
        x1, y1 = points[i-1]
        x2, y2 = points[i]
        x3, y3 = points[i+1]
        
        # Calculate curvature using three points
        # k = 4 * Area / (a * b * c) where Area is triangle area
        
        # Side lengths
        a = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        b = np.sqrt((x3-x2)**2 + (y3-y2)**2)
        c = np.sqrt((x3-x1)**2 + (y3-y1)**2)
        
        # Triangle area using cross product
        area = abs((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)) / 2.0
        
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            curvatures.append(0.0)
        else:
            k = 4 * area / (a * b * c)
            curvatures.append(k)
    
    curvatures.append(0.0)  # Last point has zero curvature
    
    return curvatures

def save_trajectory_json(points: List[Tuple[int, int]], filename: str):
    """Save trajectory to JSON file"""
    physical_coords = [pixels_to_meters(p) for p in points]
    resampled_coords = resample_path(physical_coords, RESAMPLE_SPACING)
    curvatures = calculate_curvature(resampled_coords)
    
    data = {
        "metadata": {
            "room_width_m": ROOM_WIDTH,
            "room_height_m": ROOM_HEIGHT,
            "canvas_width_px": CANVAS_WIDTH,
            "canvas_height_px": CANVAS_HEIGHT,
            "timestamp": datetime.now().isoformat(),
            "num_waypoints": len(resampled_coords),
            "waypoint_spacing_m": RESAMPLE_SPACING
        },
        "waypoints": [
            {
                "x": round(x, 4),
                "y": round(y, 4),
                "curvature": round(k, 4)
            }
            for (x, y), k in zip(resampled_coords, curvatures)
        ]
    }
    
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    
    return resampled_coords, curvatures

def save_trajectory_csv(resampled_coords: List[Tuple[float, float]], 
                        curvatures: List[float], filename: str):
    """Save trajectory to CSV file for easy import"""
    with open(filename, 'w') as f:
        f.write("index,x_m,y_m,curvature\n")
        for i, ((x, y), k) in enumerate(zip(resampled_coords, curvatures)):
            f.write(f"{i},{x:.4f},{y:.4f},{k:.4f}\n")

def save_trajectory_cpp_header(resampled_coords: List[Tuple[float, float]], 
                                curvatures: List[float], filename: str):
    """Generate C++ header file with trajectory data"""
    with open(filename, 'w') as f:
        f.write("// Auto-generated trajectory waypoints\n")
        f.write(f"// Generated: {datetime.now().isoformat()}\n")
        f.write(f"// Room dimensions: {ROOM_WIDTH}m x {ROOM_HEIGHT}m\n")
        f.write(f"// Number of waypoints: {len(resampled_coords)}\n\n")
        
        f.write("#ifndef TRAJECTORY_WAYPOINTS_HPP\n")
        f.write("#define TRAJECTORY_WAYPOINTS_HPP\n\n")
        
        f.write("#include <vector>\n")
        f.write("#include <array>\n\n")
        
        f.write("namespace trajectory {\n\n")
        
        # Waypoint structure
        f.write("struct Waypoint {\n")
        f.write("    double x;          // meters\n")
        f.write("    double y;          // meters\n")
        f.write("    double curvature;  // 1/meters\n")
        f.write("};\n\n")
        
        # Trajectory data
        f.write(f"constexpr size_t NUM_WAYPOINTS = {len(resampled_coords)};\n")
        f.write(f"constexpr double WAYPOINT_SPACING = {RESAMPLE_SPACING};\n\n")
        
        f.write("const std::vector<Waypoint> waypoints = {\n")
        for (x, y), k in zip(resampled_coords, curvatures):
            f.write(f"    {{{x:.4f}, {y:.4f}, {k:.4f}}},\n")
        f.write("};\n\n")
        
        f.write("} // namespace trajectory\n\n")
        f.write("#endif // TRAJECTORY_WAYPOINTS_HPP\n")

def save_trajectory(points: List[Tuple[int, int]], base_filename: str = None):
    """Save trajectory in multiple formats for C++ pure pursuit"""
    if not points:
        print("No trajectory to save!")
        return
    
    if base_filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"trajectory_{timestamp}"
    
    # Remove extension if provided
    if '.' in base_filename:
        base_filename = base_filename.rsplit('.', 1)[0]
    
    print(f"\n{'='*70}")
    print(f"Saving trajectory...")
    print(f"{'='*70}")
    
    # Save JSON
    resampled_coords, curvatures = save_trajectory_json(points, f"{base_filename}.json")
    print(f"✓ JSON saved: {base_filename}.json")
    
    # Save CSV
    if EXPORT_CSV:
        save_trajectory_csv(resampled_coords, curvatures, f"{base_filename}.csv")
        print(f"✓ CSV saved: {base_filename}.csv")
    
    # Save C++ header
    if EXPORT_CPP_HEADER:
        save_trajectory_cpp_header(resampled_coords, curvatures, f"{base_filename}.hpp")
        print(f"✓ C++ header saved: {base_filename}.hpp")
    
    print(f"\n{'='*70}")
    print(f"Trajectory Statistics:")
    print(f"{'='*70}")
    print(f"Room dimensions: {ROOM_WIDTH}m x {ROOM_HEIGHT}m")
    print(f"Number of waypoints: {len(resampled_coords)}")
    print(f"Waypoint spacing: {RESAMPLE_SPACING}m")
    print(f"Total path length: {calculate_path_length(resampled_coords):.2f}m")
    print(f"Max curvature: {max(curvatures):.4f} (1/m)")
    print(f"\nFirst 10 waypoints (x, y, curvature):")
    print(f"{'Index':<8} {'X (m)':<10} {'Y (m)':<10} {'Curvature':<12}")
    print(f"{'-'*45}")
    
    for i, ((x, y), k) in enumerate(zip(resampled_coords[:10], curvatures[:10])):
        print(f"{i:<8} {x:<10.4f} {y:<10.4f} {k:<12.4f}")
    
    if len(resampled_coords) > 10:
        print(f"... ({len(resampled_coords) - 10} more waypoints)")
    
    print(f"{'='*70}\n")

def calculate_path_length(coords: List[Tuple[float, float]]) -> float:
    """Calculate total path length"""
    length = 0.0
    for i in range(1, len(coords)):
        x1, y1 = coords[i-1]
        x2, y2 = coords[i]
        length += np.sqrt((x2-x1)**2 + (y2-y1)**2)
    return length

# ============================================================================
# Main GUI Application
# ============================================================================

class TrajectoryDrawer:
    def __init__(self):
        pygame.init()
        
        # Create window
        self.screen = pygame.display.set_mode((CANVAS_WIDTH, CANVAS_HEIGHT + 100))
        pygame.display.set_caption("Trajectory Drawer - Draw your path!")
        
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        # Drawing state
        self.drawing = False
        self.raw_points = []  # All points while drawing
        self.trajectory = []   # Smoothed final trajectory
        
        # Button rectangles
        self.clear_button = pygame.Rect(20, CANVAS_HEIGHT + 20, 120, 50)
        self.save_button = pygame.Rect(160, CANVAS_HEIGHT + 20, 120, 50)
        self.quit_button = pygame.Rect(300, CANVAS_HEIGHT + 20, 120, 50)
        
    def draw_grid(self):
        """Draw grid lines for visual reference"""
        if not SHOW_GRID:
            return
        
        # Calculate grid spacing in pixels
        grid_spacing_x = (GRID_SPACING_METERS / ROOM_WIDTH) * CANVAS_WIDTH
        grid_spacing_y = (GRID_SPACING_METERS / ROOM_HEIGHT) * CANVAS_HEIGHT
        
        # Vertical lines
        x = grid_spacing_x
        while x < CANVAS_WIDTH:
            pygame.draw.line(self.screen, GRID_COLOR, (int(x), 0), (int(x), CANVAS_HEIGHT), 1)
            x += grid_spacing_x
        
        # Horizontal lines
        y = grid_spacing_y
        while y < CANVAS_HEIGHT:
            pygame.draw.line(self.screen, GRID_COLOR, (0, int(y)), (CANVAS_WIDTH, int(y)), 1)
            y += grid_spacing_y
    
    def draw_canvas(self):
        """Draw the main canvas area"""
        # Background
        canvas_rect = pygame.Rect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT)
        pygame.draw.rect(self.screen, BACKGROUND_COLOR, canvas_rect)
        
        # Grid
        self.draw_grid()
        
        # Border
        pygame.draw.rect(self.screen, (0, 0, 0), canvas_rect, 2)
        
        # Draw trajectory line
        if len(self.trajectory) > 1:
            pygame.draw.lines(self.screen, DRAW_COLOR, False, self.trajectory, LINE_WIDTH)
        
        # Draw waypoint markers
        for point in self.trajectory:
            pygame.draw.circle(self.screen, WAYPOINT_COLOR, point, POINT_RADIUS)
            # Draw small cross at waypoint
            pygame.draw.line(self.screen, (255, 255, 255), 
                           (point[0]-3, point[1]), (point[0]+3, point[1]), 2)
            pygame.draw.line(self.screen, (255, 255, 255), 
                           (point[0], point[1]-3), (point[0], point[1]+3), 2)
    
    def draw_buttons(self):
        """Draw control buttons"""
        button_color = (70, 70, 70)
        hover_color = (100, 100, 100)
        text_color = (255, 255, 255)
        
        mouse_pos = pygame.mouse.get_pos()
        
        # Clear button
        color = hover_color if self.clear_button.collidepoint(mouse_pos) else button_color
        pygame.draw.rect(self.screen, color, self.clear_button, border_radius=5)
        pygame.draw.rect(self.screen, (0, 0, 0), self.clear_button, 2, border_radius=5)
        text = self.font.render("Clear", True, text_color)
        text_rect = text.get_rect(center=self.clear_button.center)
        self.screen.blit(text, text_rect)
        
        # Save button
        color = hover_color if self.save_button.collidepoint(mouse_pos) else button_color
        if not self.trajectory:
            color = (50, 50, 50)  # Disabled
        pygame.draw.rect(self.screen, color, self.save_button, border_radius=5)
        pygame.draw.rect(self.screen, (0, 0, 0), self.save_button, 2, border_radius=5)
        text = self.font.render("Save", True, text_color)
        text_rect = text.get_rect(center=self.save_button.center)
        self.screen.blit(text, text_rect)
        
        # Quit button
        color = hover_color if self.quit_button.collidepoint(mouse_pos) else button_color
        pygame.draw.rect(self.screen, color, self.quit_button, border_radius=5)
        pygame.draw.rect(self.screen, (0, 0, 0), self.quit_button, 2, border_radius=5)
        text = self.font.render("Quit", True, text_color)
        text_rect = text.get_rect(center=self.quit_button.center)
        self.screen.blit(text, text_rect)
    
    def draw_info(self):
        """Draw information text"""
        info_y = CANVAS_HEIGHT + 75
        
        # Room dimensions
        info_text = f"Room: {ROOM_WIDTH}m × {ROOM_HEIGHT}m | Points: {len(self.trajectory)}"
        text = self.small_font.render(info_text, True, (0, 0, 0))
        self.screen.blit(text, (440, info_y))
        
        # Instructions
        if not self.trajectory:
            instruction = "Click and drag to draw your trajectory"
        else:
            instruction = "Draw complete - Click SAVE to export"
        
        text = self.small_font.render(instruction, True, (0, 100, 0))
        self.screen.blit(text, (440, info_y + 20))
    
    def handle_mouse_down(self, pos):
        """Handle mouse button down"""
        x, y = pos
        
        # Check if clicking on canvas
        if y < CANVAS_HEIGHT:
            self.drawing = True
            self.raw_points = [pos]
        
        # Check button clicks
        if self.clear_button.collidepoint(pos):
            self.raw_points = []
            self.trajectory = []
            print("Trajectory cleared")
        
        elif self.save_button.collidepoint(pos) and self.trajectory:
            save_trajectory(self.trajectory)
        
        elif self.quit_button.collidepoint(pos):
            return False
        
        return True
    
    def handle_mouse_up(self, pos):
        """Handle mouse button up"""
        if self.drawing:
            self.drawing = False
            # Smooth and process the path
            smoothed = smooth_path(self.raw_points, 5)  # Initial smoothing
            
            # Convert to physical coordinates
            physical_coords = [pixels_to_meters(p) for p in smoothed]
            
            # Resample for uniform spacing (critical for pure pursuit)
            resampled_physical = resample_path(physical_coords, RESAMPLE_SPACING)
            
            # Convert back to pixels for display
            self.trajectory = [meters_to_pixels(p) for p in resampled_physical]
            
            print(f"Path processed: {len(self.raw_points)} raw -> " +
                  f"{len(smoothed)} smoothed -> {len(self.trajectory)} waypoints " +
                  f"(spacing: {RESAMPLE_SPACING}m)")
            print(f"Total path length: {calculate_path_length(resampled_physical):.2f}m")
    
    def handle_mouse_motion(self, pos):
        """Handle mouse motion"""
        x, y = pos
        
        if self.drawing and y < CANVAS_HEIGHT:
            # Add point if it's far enough from the last one (avoid duplicate points)
            if not self.raw_points or \
               abs(self.raw_points[-1][0] - x) > 2 or abs(self.raw_points[-1][1] - y) > 2:
                self.raw_points.append(pos)
    
    def run(self):
        """Main application loop"""
        print("="*70)
        print("Trajectory Drawer for Pure Pursuit Algorithm")
        print("="*70)
        print(f"Room dimensions: {ROOM_WIDTH}m × {ROOM_HEIGHT}m")
        print(f"Canvas size: {CANVAS_WIDTH}px × {CANVAS_HEIGHT}px")
        print(f"Waypoint spacing: {RESAMPLE_SPACING}m")
        print("\nInstructions:")
        print("  - Click and drag to draw your desired trajectory")
        print("  - Release to finish drawing (auto-resamples for pure pursuit)")
        print("  - Click 'Clear' to start over")
        print("  - Click 'Save' to export in multiple formats:")
        print("    * JSON (metadata + waypoints)")
        print("    * CSV (easy import to any tool)")
        print("    * HPP (C++ header file - ready to #include)")
        print("  - Click 'Quit' or close window to exit")
        print("="*70 + "\n")
        
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if not self.handle_mouse_down(event.pos):
                        running = False
                
                elif event.type == pygame.MOUSEBUTTONUP:
                    self.handle_mouse_up(event.pos)
                
                elif event.type == pygame.MOUSEMOTION:
                    self.handle_mouse_motion(event.pos)
            
            # Draw everything
            self.screen.fill((255, 255, 255))
            self.draw_canvas()
            self.draw_buttons()
            self.draw_info()
            
            pygame.display.flip()
            self.clock.tick(60)  # 60 FPS
        
        pygame.quit()
        print("\nTrajectory Drawer closed")

# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    app = TrajectoryDrawer()
    app.run()