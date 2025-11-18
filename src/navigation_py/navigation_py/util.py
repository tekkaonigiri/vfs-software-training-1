import curses
import numpy as np
import rclpy

# screen is 400 x 400
SIMSIZE = 400
# TARGET_ROW_PIXEL = -1
# TARGET_COL_PIXEL = -1

# rounding data to utiliy func
# translate floats into screen size user is looking at
# be able to take data given in floats and round that to 
# put onto screen for user to see + understand
# loc_data [x, y] --> need to normalize into actual screen index (row, column)
# def get_coords(loc_data: np.ndarray, height, width):

#     # normalize
#     normalize_x = loc_data[0] / MAX_SCREEN
#     normalize_y = loc_data[1] / MAX_SCREEN
#     # round
#     row = int( round( normalize_x * (height - 1) ) )
#     col = int( round( normalize_y * (width - 1) ) )

#     return row, col

# def test_scr(screen):
#     screen.addstr(5,5,"test")
#     screen.refresh()

def update_screen(drone_loc: np.ndarray, target_loc: np.ndarray, screen):

    height, width = screen.getmaxyx()
    SCREENSIZE = min([height, width, 400])
    drone_row = int(SCREENSIZE/SIMSIZE * drone_loc[1])
    drone_col = int(SCREENSIZE/SIMSIZE * drone_loc[0])
    target_row = int(SCREENSIZE/SIMSIZE * target_loc[1])
    target_col = int(SCREENSIZE/SIMSIZE * target_loc[0])
    screen.erase()

    # TODO:
    # figure out how to draw on the screen
    # maybe do try except error?? put nested for loop inside of it

    # loop through + if i, j indexes equal drone point / target point
        # then print yay
        # change screen icon at that point accordingly
        # else print some symbol "-"
    # is i and j row and col??
    for i in range(SCREENSIZE):
        for j in range(SCREENSIZE):
            if i == drone_row and j == drone_col:
                screen.addstr(i, j, 'H')
            elif i == target_row and j == target_col:
                screen.addstr(i,j, '0')
            else:
                screen.addstr(i, j, '-')

    # screen.addstr(0, 0, f"Terminal size: {height}x{width}")
    # screen.addstr(1, 0, f"Drone: Row = {drone_row}, Column = {drone_col}")
    # screen.addstr(1, 0, 
    #     f"Target: Row = {TARGET_ROW_PIXEL}, Column = {TARGET_COL_PIXEL}")

    screen.refresh()


# def main(screen):
#     height, width = screen.getmaxyx()

#     screen.addstr(0, 0, f"Screen Height: {height}")
#     screen.addstr(1, 0, f"Screen Width: {width}")
#     screen.refresh()

#     if curses.is_term_resized(height, width):
#         curses.resize(height, width)
#         screen.clear()

    # TODO:
    # some while loop here to cotinue updating the drone
        # should be able to read the data (pos and vel) from ros2 topics
        # call update screen to draw the screen based on drones new position

        # pause func to not overload the screen bc of the constant info
            # i think curses.napms(x) could work or like time.sleep(x)

        # add some sort of key or quit button to get out
            # use screen.getch()
            # wait for user input before exiting

#     # 0,0 coordinates for string
#     screen.addstr(0, 0, f"Terminal size: {height}x{width}") 

# def run():
#     curses.wrapper(main)