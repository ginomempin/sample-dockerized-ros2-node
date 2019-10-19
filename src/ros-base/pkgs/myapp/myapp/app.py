"""
Service App

All implementations related to app/service should be here.
"""


###############################################################################
# IMPORTS
###############################################################################


from random import randint


###############################################################################
# CONSTANTS
###############################################################################


###############################################################################
# PUBLIC
###############################################################################


class UserData():
    def __init__(self, user_id: int):
        self.x = randint(2, 4)
        self.y = randint(6, 8)
        self.z = randint(9, 12)


def apply_command(x, y, z, action):
    print(f"APP applied command with params x={x},y={y},z={z},action={action}")


def get_data(user_id: int):
    print(f"APP is getting data for user={user_id}")

    return UserData(user_id)
