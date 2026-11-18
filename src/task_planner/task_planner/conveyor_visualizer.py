import pygame
import numpy as np
import os
import json
from collections import deque
import random

#i set this up as a seperate file so it can be imported into stuff
class ConveyorVisualizer:

    def __init__(self, env, cell_size=25):

        pygame.init()

        self.env = env

        #cell size affects the visual size
        #if its too high it might overflow over the computer screen
        self.cell_size = cell_size

        #these represent the width and length of hte conveyor itself
        self.width = env.width
        self.length = env.length

        #more visual dimensions
        self.margin = 160
        self.panel_width = 260
        
        #auto calculated via the prior parameters, just leave alone
        self.screen_width = (
            self.length * cell_size
            + self.panel_width
            + self.margin * 3
        )

        self.screen_height = (
            self.width * cell_size
            + self.margin * 2
        )

        self.screen = pygame.display.set_mode(
            (self.screen_width, self.screen_height)
        )

        pygame.display.set_caption(
            "Conveyor Sorting Visualiser"
        )

        self.clock = pygame.time.Clock()
        
        #save button
        self.save_button = pygame.Rect(
            self.screen_width - self.panel_width - 20,
            self.screen_height - 65,
            self.panel_width,
            40
        )

        #font setup, can be changed if needed
        #i suppose the current setup isnt very "professional"
        self.font = pygame.font.SysFont(
            "comicsansms", 18
        )

        self.small_font = pygame.font.SysFont(
            "comicsansms", 13
        )

        self.title_font = pygame.font.SysFont(
            "comicsansms", 24, bold=True
        )

        #color for each object type
        self.type_colours = [
            (50, 140, 255), #0 - glass
            (50, 200, 100), #1 - metal
            (255, 190, 40), #2 - paper
            (230, 65, 65), #3 - cardboard
            (255, 192, 203), #4 plastic
        ]

        #image paths for each object type
        #replace these placeholders with the paths to your images
        #images should ideally have transparent backgrounds (PNG)
        self.type_image_paths = {
            0: "images/glass.png",
            1: "images/metal.png",
            2: "images/cardboard.png",
            3: "images/plastic.png",
        }

        #load the images and resize them to fit the conveyor
        self.type_images = {}

        #size of the object images on the conveyor
        self.object_image_size = (
            int(self.cell_size * 0.9),
            int(self.cell_size * 0.9),
        )
        
        #way to store the last 100 states
        self.state_history = deque(maxlen = 100)

        for object_type, image_path in self.type_image_paths.items():

            if os.path.exists(image_path):

                image = pygame.image.load(
                    image_path
                ).convert_alpha()

                image = pygame.transform.smoothscale(
                    image,
                    self.object_image_size,
                )

                self.type_images[object_type] = image

            else:

                print(
                    f"Warning: Image not found for type "
                    f"{object_type}: {image_path}"
                )

        self.background = (30, 63, 40)
        self.conveyor_colour = (75, 68, 75)
        self.grid_colour = (90, 93, 100)

        self.running = True

    #helper function to write text at a position
    def draw_text(self, text, x, y, font=None, colour=(255, 255, 255)):

        if font is None:
            font = self.font

        surface = font.render(
            str(text), True, colour
        )

        self.screen.blit(surface, (x, y))

    #main function to draw the conveyor
    def draw_conveyor(self):

        start_x = self.margin
        start_y = self.margin

        conveyor_width = self.length * self.cell_size
        conveyor_height = self.width * self.cell_size

        #conveyor background
        pygame.draw.rect(
            self.screen,
            self.conveyor_colour,
            (
                start_x,
                start_y,
                conveyor_width,
                conveyor_height,
            ),
            border_radius=8,
        )

        #draw grid
        for x in range(self.length + 1):

            px = start_x + x * self.cell_size

            pygame.draw.line(
                self.screen,
                self.grid_colour,
                (px, start_y),
                (px, start_y + conveyor_height),
                1,
            )

        for y in range(self.width + 1):

            py = start_y + y * self.cell_size

            pygame.draw.line(
                self.screen,
                self.grid_colour,
                (start_x, py),
                (start_x + conveyor_width, py),
                1,
            )

        #conveyor boundaries
        pygame.draw.rect(
            self.screen,
            (180, 180, 180),
            (
                start_x,
                start_y,
                conveyor_width,
                conveyor_height,
            ),
            3,
            border_radius=8,
        )

    def draw_robot_zones(self):

        zones = [
            (
                self.env.arm_positions,
                (60, 130, 255),
                "Arm",
            ),
            (
                self.env.gantry_positions,
                (180, 80, 255),
                "Gantry",
            ),
        ]

        for positions, colour, name in zones:

            if len(positions) == 0:
                continue

            min_x = positions[:, 0].min()
            max_x = positions[:, 0].max()

            min_y = positions[:, 1].min()
            max_y = positions[:, 1].max()

            #convert the original environment coordinates into horizontal screen coordinates
            px = (
                self.margin
                + (self.length - 1 - max_y) * self.cell_size
            )

            py = (
                self.margin
                + (min_x - 1) * self.cell_size
            )

            rect_width = (
                (max_y - min_y + 1) * self.cell_size
            )

            rect_height = (
                (max_x - min_x + 1) * self.cell_size
            )

            zone_surface = pygame.Surface(
                (rect_width, rect_height),
                pygame.SRCALPHA,
            )

            zone_surface.fill(
                (*colour, 45)
            )

            self.screen.blit(
                zone_surface,
                (px, py),
            )

            pygame.draw.rect(
                self.screen,
                colour,
                (
                    px,
                    py,
                    rect_width,
                    rect_height,
                ),
                2,
            )

            self.draw_text(
                name,
                px + 5,
                py + 5,
                self.small_font,
                colour,
            )

    #helper function that creates a snapshot of the current state for saving purposes
    def capture_state(self):
        return {
            "step_count": self.env.step_count,

            "items": [
                {
                    "x": item.x,
                    "y": item.y,
                    "type": item.type
                }
                for item in self.env.items
            ],

            "arm_positions": self.env.arm_positions.tolist(),
            "gantry_positions": self.env.gantry_positions.tolist(),

            #"total_correct": self.env.total_correct,
            #"total_wrong": self.env.total_wrong,
            #"total_failed_picks": self.env.total_failed_picks,
            #"total_missed": self.env.total_missed,

            #"arm_cooldown": self.env.arm.cooldown,
            #"gantry_cooldown": self.env.gantry.cooldown
        }
    
    
    #function to save the current recording to a json
    def save_states(self):
        data = {
            "width": self.env.width,
            "length": self.env.length,
            "states": list(self.state_history)
        }

        #random number to append to file name to save multiple
        random_num = random.randint(1, 100000)
        
        with open("./conveyor_recording_" + str(random_num) + ".json", "w") as file:

            json.dump(data, file, indent=4)

        print(
            f"Saved {len(self.state_history)} steps."
        )
    
    #helper function to draw objects on the conveyor
    def draw_objects(self):

        for index, item in enumerate(self.env.items):

            #grid to screen coordinate conversion
            px = (
                self.margin
                + (self.length - 1 - item.y) * self.cell_size
                + self.cell_size // 2
            )

            py = (
                self.margin
                + item.x * self.cell_size
                + self.cell_size // 2
            )

            colour = self.type_colours[
                item.type % len(self.type_colours)
            ]

            #draw the object using its corresponding image
            if item.type in self.type_images:

                image = self.type_images[
                    item.type
                ]

                image_rect = image.get_rect(
                    center=(px, py)
                )

                self.screen.blit(
                    image,
                    image_rect,
                )

            else:

                #fallback to the original circle if an image is missing
                radius = self.cell_size // 3

                pygame.draw.circle(
                    self.screen,
                    colour,
                    (px, py),
                    radius,
                )

                pygame.draw.circle(
                    self.screen,
                    (255, 255, 255),
                    (px, py),
                    radius,
                    2,
                )

            #label for the object
            #label = self.small_font.render(
            #    str(index),
            #    True,
            #    (255, 255, 255),
            #)


            #self.screen.blit(
            #    label,
            #    (
            #        px - label.get_width() // 2,
            #        py - label.get_height() // 2,
            #    ),
            #)

    #alternate draw object function to draw objects that are getting picked by a robot
    def draw_selected_objects(self, action):

        if action is None:
            return

        arm_action = int(action[0])
        gantry_action = int(action[1])

        arm_indices = []
        gantry_indices = []

        if arm_action > 0:
            arm_indices.append(arm_action - 1)

        if gantry_action > 0:
            gantry_indices.append(gantry_action - 1)

        #remove duplicate selections
        arm_indices = set(arm_indices)
        gantry_indices = set(gantry_indices)

        for index in arm_indices:

            if index >= len(self.env.items):
                continue

            item = self.env.items[index]

            px = (
                self.margin
                + (self.length - 1 - item.y) * self.cell_size
            )

            py = (
                self.margin
                + item.x * self.cell_size
            )

            pygame.draw.rect(
                self.screen,
                (100, 130, 255),
                (
                    px + 1,
                    py + 1,
                    self.cell_size - 2,
                    self.cell_size - 2,
                ),
                3,
            )
        
        for index in gantry_indices:

            if index >= len(self.env.items):
                continue

            item = self.env.items[index]

            px = (
                self.margin
                + (self.length - 1 - item.y) * self.cell_size
            )

            py = (
                self.margin
                + item.x * self.cell_size
            )

            pygame.draw.rect(
                self.screen,
                (210, 80, 255),
                (
                    px + 1,
                    py + 1,
                    self.cell_size - 2,
                    self.cell_size - 2,
                ),
                3,
            )

    #helper function to append the state to the state list
    def record_state(self):
        self.state_history.append(
            self.capture_state()
        )
    
    #draws the side panel information with all the statistics
    def draw_information(self):
        #panel outline
        panel_x = (
            self.margin * 2
            + self.length * self.cell_size
        )

        y = 0
        #label text
        self.draw_text(
            "SYSTEM STATUS",
            panel_x,
            y,
            self.title_font,
        )

        y += 55

        #all the statistics
        stats = [
            f"Step: {self.env.step_count}",
            f"Objects: {len(self.env.items)}",
            "",
            #f"Correct sorts: {self.env.total_correct}",
            #f"Wrong sorts: {self.env.total_wrong}",
            #f"Failed picks: {self.env.total_failed_picks}",
            #f"Missed objects: {self.env.total_missed}",
            #"",
            #f"Arm cooldown: {self.env.arm.cooldown}",
            #f"Gantry cooldown: {self.env.gantry.cooldown}",
        ]

        #draw each stat, the stat list can be updated
        for stat in stats:

            self.draw_text(
                stat,
                panel_x,
                y,
            )

            y += 30

        y += 20

        self.draw_text(
            "OBJECT TYPES",
            panel_x,
            y,
            self.title_font,
        )

        y += 45

        #draw the legend for each object type
        for i, colour in enumerate(self.type_colours):

            pygame.draw.circle(
                self.screen,
                colour,
                (panel_x + 10, y + 10),
                9,
            )

            self.draw_text(
                f"Type {i}",
                panel_x + 30,
                y,
            )

            y += 30

    #helper function to draw the save button
    def draw_save_button(self):
        pygame.draw.rect(
            self.screen,
            (45, 130, 75),
            self.save_button,
            border_radius=8
        )

        pygame.draw.rect(
            self.screen,
            (100, 200, 130),
            self.save_button,
            2,
            border_radius=8
        )

        text = self.font.render(
            "Save Last 100 Steps",
            True,
            (255, 255, 255)
        )

        text_rect = text.get_rect(
            center=self.save_button.center
        )

        self.screen.blit(text, text_rect)
    
    #runs all of the above functions, call this from whichever file it is imported into to do everything
    def render(self, action=None, record = True):
        if record:
            self.record_state()
        
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                self.running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.save_button.collidepoint(event.pos):
                    self.save_states()

        self.screen.fill(self.background)

        self.draw_conveyor()

        self.draw_robot_zones()

        self.draw_objects()

        self.draw_selected_objects(action)

        self.draw_information()

        self.draw_save_button()
        
        pygame.display.flip()

        self.clock.tick(30)

    def close(self):

        pygame.quit()
