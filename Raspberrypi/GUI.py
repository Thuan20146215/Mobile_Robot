import tkinter as tk
from tkinter import ttk
import time
import os

point_coordinates = {
    1: (94 // 18, 94 % 18),
    2: (99 // 18, 99 % 18),
    3: (104 // 18, 104 % 18),
    4: (148 // 18, 148 % 18),
    5: (153 // 18, 153 % 18),
    6: (158 // 18, 158 % 18),
    7: (202 // 18, 202 % 18),
    8: (207 // 18, 207 % 18),
    9: (212 // 18, 212 % 18),
    10: (132 // 18, 132 % 18),
    11: (137 // 18, 137 % 18),
    12: (186 // 18, 186 % 18),
    13: (191 // 18, 191 % 18),
}

matrix = []  # Global variable for matrix

def read_map(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        size = list(map(int, lines[0].strip().split()))
        matrix = [line.strip().split() for line in lines[1:]]
        return size, matrix

def read_path(file_path):
    if not os.path.exists(file_path):
        return None, 0
    with open(file_path, 'r') as file:
        lines = file.readlines()
        point = int(lines[0].strip())
        color1_value = int(lines[1].strip())
        return point_coordinates.get(point), color1_value

def calculate_path(start, end, matrix):
    from collections import deque
    
    rows, cols = len(matrix), len(matrix[0])
    directions = [(0,1), (1,0), (0,-1), (-1,0)]
    
    queue = deque([start])
    visited = {start: None}
    
    while queue:
        current = queue.popleft()
        
        if current == end:
            break
        
        for direction in directions:
            next_row = current[0] + direction[0]
            next_col = current[1] + direction[1]
            
            if (0 <= next_row < rows and 0 <= next_col < cols and 
                matrix[next_row][next_col] == '.' and (next_row, next_col) not in visited):
                queue.append((next_row, next_col))
                visited[(next_row, next_col)] = current
    
    path = []
    step = end
    while step is not None:
        path.append(step)
        step = visited[step]
    path.reverse()
    return path

def display_map(size, matrix, delay=0.2):
    global root, labels, first_color_box  # Make root, labels, and first_color_box global variables

    root = tk.Tk()
    root.title("Map Visualization")

    # Set the window size to a suitable value
    root.geometry("1000x400")

    # Create main frame to hold everything
    main_frame = tk.Frame(root)
    main_frame.pack(fill="both", expand=True)

    # Configure row and column weights for resizing
    main_frame.columnconfigure(0, weight=3)
    main_frame.columnconfigure(1, weight=1)
    main_frame.rowconfigure(0, weight=1)

    # Create left frame for the map with reduced padding
    map_frame = tk.Frame(main_frame, relief="sunken", borderwidth=1)
    map_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")

    # Create right frame for the controls with reduced padding
    control_frame = tk.Frame(main_frame, relief="sunken", borderwidth=1)
    control_frame.grid(row=0, column=1, padx=5, pady=5, sticky="n")

    # Set canvas size to a more compact display
    canvas = tk.Canvas(map_frame, width=450, height=300)
    canvas.pack(fill="both", expand=True)

    # Create a frame inside the canvas which will be scrolled
    inner_frame = tk.Frame(canvas)
    canvas.create_window((0, 0), window=inner_frame, anchor="nw")

    labels = [[None for _ in range(size[1])] for _ in range(size[0])]

    def create_map_labels(matrix):
        for i in range(size[0]):
            for j in range(size[1]):
                color = "black" if matrix[i][j] == '@' else "white"
                cell = tk.Label(inner_frame, bg=color, width=2, height=1, borderwidth=1, relief="solid")
                cell.grid(row=i, column=j, padx=0.5, pady=0.5)  # Reduce padding between cells
                labels[i][j] = cell

    create_map_labels(matrix)

    def reload_map():
        nonlocal matrix  # Use nonlocal matrix variable
        size, matrix = read_map(map_file_path)
        update_map_display(matrix)  # Update the display with new matrix
        return matrix

    def update_map_display(new_matrix):
        for i in range(size[0]):
            for j in range(size[1]):
                color = "black" if new_matrix[i][j] == '@' else "white"
                if labels[i][j]:
                    labels[i][j].configure(bg=color)

    reload_button = tk.Button(control_frame, text="Reload Map", command=reload_map)
    reload_button.grid(row=0, column=0, columnspan=2, pady=5)

    def update_path():
        nonlocal current_position

        target, color1_value = read_path(path_file_path)

        if target is None:
            root.after(1000, update_path)
            return

        current_matrix = reload_map()

        segment = calculate_path(current_position, target, current_matrix)

        colors1 = ["red", "green", "yellow"]
        color1_names = {2: "ERROR", 1: "RUNNING...", 0: "STOP"}

        first_box_color = colors1[color1_value] if color1_value in [0, 1, 2] else "white"
        first_color_name = color1_names.get(color1_value, "Unknown")

        first_color_box.config(bg=first_box_color, text=first_color_name,font=("Times New Roman", 14, "bold italic"))

        if current_position and target:
            for x, y in segment:
                if 0 <= x < size[0] and 0 <= y < size[1] and current_matrix[x][y] == '.':
                    if labels[x][y]:
                        labels[x][y].configure(bg='red')
                    root.update_idletasks()
                    time.sleep(delay)
                    if (x, y) != segment[-1]:
                        if labels[x][y]:
                            labels[x][y].configure(bg='white')
                else:
                    print(f"Invalid path coordinates: ({x}, {y})")
            current_position = target
            print("Car movement complete")
        else:
            print("No valid path found")

        root.after(1000, update_path)

    current_position = point_coordinates[1]
    if current_position:
        labels[current_position[0]][current_position[1]].configure(bg='red')

    root.after(1000, update_path)
    text_label = tk.Label(control_frame, text="HCMUTE - CĐT 25", font=("Times New Roman", 14, "bold italic"))
    text_label.grid(row=5, column=0, columnspan=2, pady=5)
    text_label1 = tk.Label(control_frame, text="Nghiên cứu thuật toán tối ưu hóa quỹ đạo cho Robot trong nhà kho", font=("Times New Roman", 14, "bold italic"))
    text_label1.grid(row=6, column=0, columnspan=2, pady=5)
    first_line_label = tk.Label(control_frame, text="")
    first_line_label.grid(row=1, column=0, columnspan=2, pady=5)

    def update_first_line_label():
        with open(path_file_path, 'r') as file:
            first_line = file.readline().strip()
        first_line_label.config(text=f"STATION: {first_line}")
        root.after(1000, update_first_line_label)

    update_first_line_label()

    first_color_box = tk.Label(control_frame, text="Box 1", width=20, height=2)
    first_color_box.grid(row=2, column=0, columnspan=2, pady=5)

    input_frame = tk.Frame(control_frame)
    input_frame.grid(row=3, column=0, columnspan=2, pady=5)

    first_frame = tk.Frame(input_frame)
    first_frame.grid(row=0, column=0, padx=5, pady=5)

    first_label = tk.Label(first_frame, text="Number of Obstacles:")
    first_label.grid(row=0, column=0, padx=5)

    global first_input_var
    first_input_var = tk.StringVar()
    first_input = tk.Entry(first_frame, textvariable=first_input_var, width=10)
    first_input.grid(row=0, column=1, padx=5)
    first_input.bind("<KeyRelease>", on_first_input_change)

    global first_box_frame
    first_box_frame = tk.Frame(first_frame)
    first_box_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5)

    second_frame = tk.Frame(input_frame)
    second_frame.grid(row=1, column=0, padx=5, pady=5)

    second_label = tk.Label(second_frame, text="Number of Agents:")
    second_label.grid(row=0, column=0, padx=5)

    global second_input_var
    second_input_var = tk.StringVar()
    second_input = tk.Entry(second_frame, textvariable=second_input_var, width=10)
    second_input.grid(row=0, column=1, padx=5)
    second_input.bind("<KeyRelease>", on_second_input_change)

    global second_box_frame
    second_box_frame = tk.Frame(second_frame)
    second_box_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5)

    start_button = tk.Button(control_frame, text="Start", command=write_to_file)
    start_button.grid(row=4, column=0, columnspan=2, pady=10)

    canvas.update_idletasks()
    canvas.config(scrollregion=canvas.bbox("all"))

    root.mainloop()


def on_first_input_change(event):
    global first_input_var
    try:
        num_boxes = int(first_input_var.get())
        create_smaller_boxes_first(num_boxes, first_box_frame)
    except ValueError:
        pass

def on_second_input_change(event):
    global second_input_var
    try:
        num_boxes = int(second_input_var.get())
        create_smaller_boxes_second(num_boxes, second_box_frame)
    except ValueError:
        pass

def create_smaller_boxes_first(num_boxes, frame):
    for widget in frame.winfo_children():
        widget.destroy()
    for _ in range(num_boxes):
        box = tk.Entry(frame, width=5)
        box.pack(side="left", padx=2)  # Reduce spacing

def create_smaller_boxes_second(num_boxes, frame):
    for widget in frame.winfo_children():
        widget.destroy()
    for _ in range(num_boxes):
        box = tk.Entry(frame, width=5)
        box.pack(side="left", padx=2)  # Reduce spacing

def write_to_file():
    global first_input_var, second_input_var  # Ensure global variable access
    first_input_value = first_input_var.get()
    second_input_value = second_input_var.get()

    first_box_values = [box.get() for box in first_box_frame.winfo_children() if isinstance(box, tk.Entry)]
    second_box_values = [box.get() for box in second_box_frame.winfo_children() if isinstance(box, tk.Entry)]

    with open("embedded/DATN/linefl/outGUI.txt", "w") as file:
        file.write(f"{first_input_value}\n")
        for value in first_box_values:
            file.write(f"{value}\n")
        file.write(f"{second_input_value}\n")
        for value in second_box_values:
            file.write(f"{value}\n")
        file.write("1\n")

if __name__ == "__main__":
    map_file_path = 'embedded/DATN/linefl/map.txt'
    path_file_path = 'embedded/DATN/linefl/path.txt'

    size, matrix = read_map(map_file_path)
    display_map(size, matrix, delay=0.2)
