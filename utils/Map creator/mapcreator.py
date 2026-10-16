import tkinter as tk
from tkinter import messagebox
import os

# --- Configuration ---
ROWS = 150
COLS = 150
CELL_SIZE = 5 

# Element definitions mapped to display colors
ELEMENTS = {
    '#': ('Wall', 'black'),
    '.': ('Path', 'white'),
    'S': ('Survivor', 'red'),
    'E': ('Entry', 'green')
}

class ArenaDesigner:
    def __init__(self, root):
        self.root = root
        self.root.title("NIDAR AirMouse High-Res Arena Designer (10cm scale)")
        
        self.current_brush = '#'
        self.grid_data = [['.' for _ in range(COLS)] for _ in range(ROWS)]
        self.rectangles = {}

        self.setup_ui()
        self.draw_initial_grid()
        self.draw_reference_grid()

    def setup_ui(self):
        # Top toolbar for brushes
        toolbar = tk.Frame(self.root, padx=5, pady=5)
        toolbar.pack(fill=tk.X)

        tk.Label(toolbar, text="Tools:").pack(side=tk.LEFT, padx=5)

        for char, (name, color) in ELEMENTS.items():
            btn = tk.Button(
                toolbar, 
                text=name, 
                bg=color, 
                fg='white' if color == 'black' else 'black',
                command=lambda c=char: self.set_brush(c)
            )
            btn.pack(side=tk.LEFT, padx=2)

        tk.Button(toolbar, text="Clear Grid", command=self.reset_grid).pack(side=tk.RIGHT, padx=5)

        # Drawing Canvas
        self.canvas = tk.Canvas(
            self.root, 
            width=COLS * CELL_SIZE, 
            height=ROWS * CELL_SIZE, 
            bg='white'
        )
        self.canvas.pack(padx=10, pady=10)

        # Bind mouse events for drawing
        self.canvas.bind("<Button-1>", self.paint)              # Left Click to Paint
        self.canvas.bind("<B1-Motion>", self.paint)             # Left Drag to Paint
        self.canvas.bind("<Double-Button-1>", self.erase)       # Double Left-Click to Erase
        self.canvas.bind("<Button-3>", self.erase)              # Right Click to Erase
        self.canvas.bind("<B3-Motion>", self.erase)             # Right Drag to Erase

        # Bottom section for output
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=5)
        
        tk.Button(btn_frame, text="Save to File (arena_map.py)", command=self.save_to_file, font=('Arial', 12, 'bold'), bg='blue', fg='white').pack()

    def set_brush(self, char):
        self.current_brush = char

    def draw_initial_grid(self):
        # Create base cells without outlines for rendering performance
        for r in range(ROWS):
            for c in range(COLS):
                x1 = c * CELL_SIZE
                y1 = r * CELL_SIZE
                x2 = x1 + CELL_SIZE
                y2 = y1 + CELL_SIZE
                
                rect_id = self.canvas.create_rectangle(
                    x1, y1, x2, y2, 
                    fill=ELEMENTS['.'][1], 
                    outline='' 
                )
                self.rectangles[(r, c)] = rect_id

    def draw_reference_grid(self):
        # Draws a dashed line every 10 cells (which equals 1 real-world meter)
        pixels_per_meter = 10 * CELL_SIZE
        width = COLS * CELL_SIZE
        height = ROWS * CELL_SIZE

        for i in range(0, width + 1, pixels_per_meter):
            # Vertical lines
            self.canvas.create_line(i, 0, i, height, fill='gray', dash=(2, 4))
        for i in range(0, height + 1, pixels_per_meter):
            # Horizontal lines
            self.canvas.create_line(0, i, width, i, fill='gray', dash=(2, 4))

    def paint(self, event):
        # Calculate grid coordinates based on mouse position
        c = event.x // CELL_SIZE
        r = event.y // CELL_SIZE

        if 0 <= r < ROWS and 0 <= c < COLS:
            # Only update if the brush is different to save CPU cycles
            if self.grid_data[r][c] != self.current_brush:
                self.grid_data[r][c] = self.current_brush
                color = ELEMENTS[self.current_brush][1]
                self.canvas.itemconfig(self.rectangles[(r, c)], fill=color)

    def erase(self, event):
        # Reverts the clicked cell back to a standard path '.'
        c = event.x // CELL_SIZE
        r = event.y // CELL_SIZE

        if 0 <= r < ROWS and 0 <= c < COLS:
            if self.grid_data[r][c] != '.':
                self.grid_data[r][c] = '.'
                self.canvas.itemconfig(self.rectangles[(r, c)], fill=ELEMENTS['.'][1])

    def reset_grid(self):
        for r in range(ROWS):
            for c in range(COLS):
                if self.grid_data[r][c] != '.':
                    self.grid_data[r][c] = '.'
                    self.canvas.itemconfig(self.rectangles[(r, c)], fill=ELEMENTS['.'][1])

    def save_to_file(self):
        filename = "arena_map.py"
        try:
            with open(filename, "w") as f:
                f.write("arena_map = [\n")
                for row in self.grid_data:
                    row_str = "".join(row)
                    f.write(f'    "{row_str}",\n')
                f.write("]\n")
            
            filepath = os.path.abspath(filename)
            messagebox.showinfo("Success", f"Array saved successfully to:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save file:\n{str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ArenaDesigner(root)
    root.mainloop()
