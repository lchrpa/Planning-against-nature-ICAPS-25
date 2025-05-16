import random
import sys
import os
import math
import argparse

from typing import List, Tuple, Dict

class cell:

    x = -1
    y = -1
    r = False
    f = True

    def __init__(self,x,y):
      self.x=x
      self.y=y
      if x == 1 and y == 1:
          self.f = False


def generate():
    
    o = parse_arguments()

    grid = generate_grid(o['grid_width'], o['grid_height'], o['number_of_resources'])
    dangerous_columns = select_dangerous_columns(o['grid_width'], o['number_of_dangerous_columns'], o['max_consecutive_dangerous_columns'])
    number_of_ships = o['number_of_dangerous_columns'] * o['ships_in_row']

    output_file = open(o['file'], 'w') if o['file'] is not None else sys.stdout

    print_problem(output_file, grid, dangerous_columns, number_of_ships, o)
    
    if o['s'] is not None:
        with open(o['s'], 'w') as file:
            print_safe_states(file, dangerous_columns, o['grid_height'], o['grid_height'])

    if o['file'] is not None:
        output_file.close()

def print_safe_states(safe_states_file, dangerous_columns: List[int], height: int, width: int):
    for x in range(1, width+1):
        if x-1 not in dangerous_columns:
            for y in range(1, height+1):
                print("(operational a) (at a l-{}-{})".format(x,y), file=safe_states_file)

def print_problem(output_file, grid: List[cell], dangerous_columns: List[int], number_of_ships: int, o: Dict[str, int]):
    print("(define (problem AUV-problem-{}x{}-{})".format(o['grid_width'], o['grid_height'], o['behavior']), file=output_file)
    print("(:domain AUV)", file=output_file)
    print_objects(output_file, o['grid_height'], o['grid_width'], o['number_of_resources'], number_of_ships)
    print_init(output_file, grid, o['grid_height'], o['grid_width'], dangerous_columns, o['behavior'], o['ships_in_row'])
    print_goal(output_file, o['number_of_resources'])
    print(")", file=output_file)

def print_goal(output_file, resources: int):
    print("(:goal", file=output_file)
    print("(and", file=output_file)
    print("(operational a)(at a l-1-1)", file=output_file)
    for r in range(1, resources + 1):
        print("(sampled r{})".format(r), file=output_file)
    print(")", file=output_file)
    print(")", file=output_file)

def print_objects(output_file, height: int, width: int, resources: int, ships: int):
   
    print("(:objects ", file=output_file)
    for y in range(1, height + 1):
        for x in range(1, width + 1):
            print("l-{}-{}".format(x, y), end=' ', file=output_file)
    print("- location", file=output_file)
   
    for r in range(1, resources + 1):
        print("r{}".format(r), end=' ', file=output_file)
    if resources > 0:
        print("- resource", file=output_file)
    
    print("a - auv", file=output_file)
    
    for s in range(1, ships + 1):
        print("s{}".format(s), end=' ', file=output_file)
    if ships > 0:
        print("- ship", file=output_file)

    print(")", file=output_file)

def print_init(output_file, grid: List[cell], height: int, width: int, dangerous_columns: List[int], behavior: int, number_of_ships_in_column: int = 1):
    print("(:init", "(operational a)", "(at a l-1-1)", "(onsurface a)", sep='\n', file=output_file)
    
    for y in range(1, height + 1):
        for x in range(1, width + 1):
            if x < width:   
               print("(connected l-{}-{} l-{}-{})".format(x, y, x+1, y), file=output_file)
               print("(connected l-{}-{} l-{}-{})".format(x+1, y, x, y), file=output_file)
            if y < height:
               print("(connected l-{}-{} l-{}-{})".format(x, y, x, y+1), file=output_file)
               print("(connected l-{}-{} l-{}-{})".format(x, y+1, x, y), file=output_file)
    
    for i, cell_with_r in enumerate([cell for row in grid for cell in row if cell.r]):
        print("(at-res r{} l-{}-{})".format(i+1, cell_with_r.x, cell_with_r.y), file=output_file)
    
    if behavior == 1:
        print_cruising_ships(output_file, grid, height, dangerous_columns)
    elif behavior == 2:
        print_wandering_ships(output_file, grid, height, number_of_ships_in_column, dangerous_columns)

    for cell_with_s in [cell for row in grid for cell in row if cell.f]:
        print("(free l-{}-{})".format(cell_with_s.x, cell_with_s.y), file=output_file)
    
    dpt = (height-random.randint(1,height-2))/2
    
    for x in range(1, width + 1):
    	for y in range(int(dpt) +1 ,height - int(dpt) + 1):
    		print("(deep l-{}-{})".format(x,y), file=output_file)
    
    print(")", file=output_file)

def print_cruising_ships(output_file, grid: List[cell], height: int, dangerous_columns: List[int]) -> None:
    for i, column in enumerate(dangerous_columns):
        from_up_down = random.randint(0, 1)
        if from_up_down:
            start = height
            end = 1
        else:
            start = 1
            end = height
        print("(at s{} l-{}-{})".format(i+1, column+1, start), file=output_file)
        grid[start - 1][column].f = False
        #print("(exit s{} l-{}-{})".format(i+1, column+1, end), file=output_file)
        for j in range(0, height-1):
            print("(connected-ship s{} l-{}-{} l-{}-{})".format(i+1, column+1, start + ((-1)**from_up_down) * j, column+1, start + ((-1)**from_up_down) * (j+1)), file=output_file)
       # print("(outside s{})".format(i+1), file=output_file)

def print_wandering_ships(output_file, grid: List[cell], height: int, number_of_ships_in_column: int, dangerous_columns: List[int]) -> None:
    ship_number = 1
    for column in dangerous_columns:
        ship_coridor_lenghts = [2] * number_of_ships_in_column
        taken = 2 * number_of_ships_in_column
        while taken < height:
            index = random.randint(0, number_of_ships_in_column - 1)
            ship_coridor_lenghts[index] += 1
            taken += 1
        starting_cell = 1
        for ship_coridor_lenght in ship_coridor_lenghts:
            position = random.randint(starting_cell, starting_cell + ship_coridor_lenght - 1)
            print("(at s{} l-{}-{})".format(ship_number, column+1, position), file=output_file)
            grid[position-1][column].f = False
            for j in range(0, ship_coridor_lenght-1):
                print("(connected-ship s{} l-{}-{} l-{}-{})".format(ship_number, column+1, starting_cell+j, column+1, starting_cell+j+1), file=output_file)
                print("(connected-ship s{} l-{}-{} l-{}-{})".format(ship_number, column+1, starting_cell+j+1, column+1, starting_cell+j), file=output_file)
            starting_cell += ship_coridor_lenght
            ship_number += 1

def generate_grid(grid_width: int, grid_height: int, number_of_resources: int) -> List[cell]:
    grid = [[cell(x, y) for x in range(1, grid_width + 1)] for y in range(1, grid_height + 1)]
    for i in random.sample(range(1, grid_height * grid_width), k=number_of_resources):
        grid[i // grid_width][i % grid_width].r = True
    return grid

def select_dangerous_columns(grid_width, number_of_danger_columns, max_consecutive_dangerous_columns):
    
    #f = (grid_width-number_of_danger_columns)/2
    #return list(range(int(f),grid_width-int(f)))
    
    selected_neighbors = []

    while len([x for x in selected_neighbors if x == -1]) != number_of_danger_columns: # while not valid solution (there is atleast one due to asserts of inputs)
        selected_neighbors = [0] * (grid_width - 1) # -1 stands for selected, any non-negative integer stands indicates number of neighbors
        for i in range(0, number_of_danger_columns):
        
            candidates = [i for i, x in enumerate(selected_neighbors) if -1 < x < max_consecutive_dangerous_columns]
            
            if len(candidates) == 0:
                break

            selected = random.choice(candidates)
        
            index_left = selected - 1
            while index_left >= 0 and selected_neighbors[index_left] == -1:
                index_left -= 1
        
            index_right = selected + 1
            while index_right < grid_width - 1 and selected_neighbors[index_right] == -1:
                index_right += 1

            if index_left >= 0:
                selected_on_left = 0
                while index_left - selected_on_left - 1 >= 0 and selected_neighbors[index_left - selected_on_left - 1] == -1:
                    selected_on_left += 1
                selected_neighbors[index_left] = (index_right - index_left) - 1 + selected_on_left
            if index_right < grid_width - 1:
                selected_on_right = 0
                while index_right + selected_on_right + 1 < grid_width - 1 and selected_neighbors[index_right + selected_on_right + 1] == -1:
                    selected_on_right += 1
                selected_neighbors[index_right] = (index_right - index_left) - 1 + selected_on_right
            selected_neighbors[selected] = -1
    
    return [i + 1 for i, x in enumerate(selected_neighbors) if x == -1]

def parse_arguments():
    
    args = get_argparse()

    grid_height = args.grid_height
    grid_width = args.grid_width
    behavior = args.behavior
    number_of_resources = args.number_of_resources
    number_of_dangerous_columns = args.number_of_dangerous_columns
    max_consecutive_dangerous_columns = args.max_consecutive_dangerous_columns
    ships_in_column = args.ships_in_column

    assert grid_width > 0
    assert grid_height > 0
    assert 1 <= behavior <= 2
    assert 0 < number_of_resources < grid_width * grid_height
    assert 0 <= number_of_dangerous_columns < grid_width
    assert 1 <= max_consecutive_dangerous_columns
    assert number_of_dangerous_columns + math.ceil(float(number_of_dangerous_columns)/float(max_consecutive_dangerous_columns)) - 1 < grid_width - 1
    assert ships_in_column * 2 <= grid_height

    return  {
                'file': args.file,
                's': args.s,
                'grid_width': grid_width, 
                'grid_height': grid_height, 
                'behavior': behavior, 
                'number_of_resources': number_of_resources, 
                'number_of_dangerous_columns': number_of_dangerous_columns,
                'max_consecutive_dangerous_columns': max_consecutive_dangerous_columns,
                'ships_in_row': ships_in_column
            }

def get_argparse():
    parser = argparse.ArgumentParser(description='AUV problem generator with safe states support.')
    parser.add_argument('grid_width', help='width of the grid', type=int)
    parser.add_argument('grid_height', help='height of the grid', type=int)
    parser.add_argument('behavior', help='behavior of ships (1=CruisingShips, 2=WanderingShips)', type=int)
    parser.add_argument('number_of_resources', help='number of resources on the grid', type=int)
    parser.add_argument('number_of_dangerous_columns', help='number of columns with ships', type=int)
    parser.add_argument('max_consecutive_dangerous_columns', help='maximal consecutive columns w ships', type=int)
    parser.add_argument('-f', '--file', help='problem output file (stdout by default)')
    parser.add_argument('-s', help='safe states output file', type=str)
    parser.add_argument('--ships-in-column', default=1, help='number of ships in each column (default 1) (ignored when behavior=1)', type=int)
    return parser.parse_args()

if __name__ == "__main__":
    generate()
