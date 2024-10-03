from launch import LaunchDescription
from launch_ros.actions import Node # Node launch description
from ament_index_python import get_package_share_directory # Get package directory
import os # Joining paths
import xacro # XACRO utilities
import random # Randomizing objects
import math
from launch.actions import TimerAction # Launch delay timer to avoid RTO

def calc_table_coords(form, x, y):

  xy_coords = []
  if form == 4:
    xy_coords = [
      [ # First table
        6 - x - 0.15,
        3 - y - 0.3 - 0.3,
        0.35, 0.0, 0.0, 0.0
      ],
      [ # Second table
        6 - x - 0.15,
        3 - y - 0.15,
        0.35, 0.0, 0.0, 0.0
      ],
      [ # Third table
        6 - x - 0.6,
        3 - y - 0.15,
        0.35, 0.0, 0.0, 0.0
      ],
      [ # Fourth table
        6 - x - 0.6,
        3 - y - 0.3 - 0.3,
        0.35, 0.0, 0.0, 0.0
      ]
    ]

  else:
    xy_coords = [
      [ # First table
        6 - x - 0.15,
        3 - y - 0.15,
        0.35, 0.0, 0.0, 0.0
      ],
      [ # Second table
        6 - x - 0.6,
        3 - y - 0.15,
        0.35, 0.0, 0.0, 0.0
      ]
    ]
  
  return xy_coords

# Check if element is present in a 2D array
def is_element_in_2d_array(array, element):
  for row in array:
      for item in row:
          if item == element:
              return True
  return False

def generate_launch_description():

  prop_list = [] # List of all generated props
  prop_color = ["red", "green", "blue"] # Prop color variations
  prop_shape = ["square_card", "circle_card", "cube", "can"] # Prop shape variations

  # Create map of tables
  # These coordinates are from lks_arena.main.xacro
  xy_cards_map = [
    calc_table_coords(4, 1.875, 0.75),
    calc_table_coords(4, 1.875 + 2.379 + 0.6 + 0.15, 1.575),
    calc_table_coords(2, 1.8 + 0.054 + 2.271, 5.85 - 0.3),
    calc_table_coords(2, 1.8 + 0.054 + 2.271 + 0.6 + 0.15 + 1.125, 5.85 - 0.3),
    calc_table_coords(4, 1.8 + 0.054 + 2.271 + 0.6 * 2 + 0.15 * 2 + 1.125 + 0.75 + 0.054 + 1.821, 6 - 0.75 - 0.6 - 0.15),
    calc_table_coords(2, 1.8 + 0.054 + 2.271 + 0.6 * 2 + 0.15 * 2 + 1.125 + 0.75 + 0.054 + 1.821, 6 - 0.75 - 0.9 - 0.15 * 2),
    [
      [6 - (1.8 + 0.054 + 0.15) - 0.15, 3 - (6 - 0.75 - 0.6 - 0.15) - 0.3 - 0.3, 0.35, 0.0, 0.0, 0.0],
      [6 - (1.8 + 0.054 + 0.15) - 0.15, 3 - (6 - 0.75 - 0.6 - 0.15) - 0.15, 0.35, 0.0, 0.0, 0.0]
    ]
  ]

  # Create 1D array from coordinate map
  xy_cards_coord = []
  for i in range(len(xy_cards_map)):
    for j in range(len(xy_cards_map[i])):
      xy_cards_coord.append(xy_cards_map[i][j]) # Fill out z, R, P, Y arguments

  cube = 9 # Total cubes to be spawned
  can = 8 # Total cans to be spawned
  payload_total = cube + can # Amount of payload to be spawned

  cube_cards = cube # Total cube cards to be spawned
  can_cards = can # Total can cards to be spawned
  cards_total = cube_cards + can_cards # Amount of cards to be spawned

  cube_rack_origin = [-4.1085, 2.85, 0.45] # Bottom right corner of 1st level of cube rack
  cube_coords = [] # Coordinates of cubes

  for i in range(3):
    cube_coords.extend([
    [
      cube_rack_origin[0] - 0.15,
      cube_rack_origin[1],
      cube_rack_origin[2] + i * 0.399,
      0.0, 0.0, 0.0
    ],
    [
      cube_rack_origin[0] - 0.15 * 2 - 0.195,
      cube_rack_origin[1],
      cube_rack_origin[2] + i * 0.399,
      0.0, 0.0, 0.0
    ],
    [
      cube_rack_origin[0] - 0.15 * 3 - 0.195 * 2,
      cube_rack_origin[1],
      cube_rack_origin[2] + i * 0.399,
      0.0, 0.0, 0.0
    ]
  ])

  can_rack_origin = [-1.9156, 2.78, 0.5] # Bottom left corner of 1st level of can rack
  can_coords = [ # Coordinates of cans
    [
      can_rack_origin[0] - 0.2502,
      can_rack_origin[1],
      can_rack_origin[2] + 0.249,
      math.pi / 2, 0.0, 0.0
    ],
    [
      can_rack_origin[0] - 0.7506,
      can_rack_origin[1],
      can_rack_origin[2] + 0.249,
      math.pi / 2, 0.0, 0.0
    ]
  ] 

  for i in range(3):
    can_coords.append([
      can_rack_origin[0] - i * 0.5004,
      can_rack_origin[1],
      can_rack_origin[2],
      math.pi / 2, 0.0, 0.0
    ])
  
  for i in range(3):
    can_coords.append([
      can_rack_origin[0] - i * 0.5004,
      can_rack_origin[1],
      can_rack_origin[2] + 0.498,
      math.pi / 2, 0.0, 0.0
    ])
  
  # Append randomized props to prop_list
  while len(prop_list) < cards_total + payload_total:

    if cube_cards > 0:
      coord = xy_cards_coord[random.randrange(0, len(xy_cards_coord))]
      if not(is_element_in_2d_array(prop_list, coord)):
        cube_cards = cube_cards - 1
        prop_list.append([
          coord, # Card xy coordinate
          prop_color[random.randrange(0, 3)], # Card color, random
          prop_shape[0] # Card shape
        ])

    elif can_cards > 0:
      coord = xy_cards_coord[random.randrange(0, len(xy_cards_coord))]
      if not(is_element_in_2d_array(prop_list, coord)):
        can_cards = can_cards - 1
        prop_list.append([
          coord, # Card xy coordinate
          prop_color[random.randrange(0, 3)], # Card color, random
          prop_shape[1] # Card shape
        ])

    elif cube > 0:
      coord = cube_coords[random.randrange(0, len(cube_coords))]
      if not(is_element_in_2d_array(prop_list, coord)):
        cube = cube - 1
        prop_list.append([
          coord, # Cube coordinate
          prop_list[len(prop_list) - cards_total][1], # Get card n color for cube n color
          prop_shape[2]
        ])

    else:
      coord = can_coords[random.randrange(0, len(can_coords))]
      if not(is_element_in_2d_array(prop_list, coord)):
        can = can - 1
        prop_list.append([
          coord, # Can coordinate
          prop_list[len(prop_list) - cards_total][1], # Get card n color for can n color
          prop_shape[3]
        ])

  # Create node_list from prop_list
  node_list = []
  for i in range(len(prop_list)):
    prop_string = xacro.process_file(
        os.path.join(get_package_share_directory('mecabot_gz'), 
                    '../../../../world_desc/lks_arena.spawnprops.xacro'),
        mappings={"color": prop_list[i][1], "shape": prop_list[i][2]}
    ).toprettyxml(indent='  ')

    prop_spawner = Node(
      package='ros_gz_sim',
      executable='create',
      parameters=[{
        'name': "%i_%s_%s" % (i, prop_list[i][1], prop_list[i][2]),
        'world': 'empty',
        'string': prop_string,
        'x': prop_list[i][0][0],
        'y': prop_list[i][0][1],
        'z': prop_list[i][0][2],
        'R': prop_list[i][0][3],
        'P': prop_list[i][0][4],
        'Y': prop_list[i][0][5]
      }]
    )

    node_list.append(prop_spawner)

  for i in prop_list:
    print(i)

  print("%i props spawning" % len(node_list))

  node_list_delayed = []
  time_adder = 0
  separation_interval = 4
  while len(node_list) - separation_interval > -separation_interval:

    remaining_item = len(node_list) - separation_interval
    index_offset = 0

    if remaining_item >= 0:
      index_offset = separation_interval
    else:
      index_offset = abs(remaining_item)

    node_list_delayed.append(
      TimerAction(
        period=1.0 + time_adder,
        actions=node_list[0:index_offset]
      )
    )

    if remaining_item >= 0:
      node_list = node_list[index_offset:]
    else:
      node_list = []

    time_adder += 1
    print(remaining_item)
    print(len(node_list))

  return LaunchDescription(node_list_delayed)