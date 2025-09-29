from mcp.server.fastmcp import FastMCP

mcp = FastMCP("Prompts")

PROMPTS = {
    "real_world": (
        "## Import Jenga Blocks from Real World Topics\n"
        "1. Get topics and read Jenga block poses.\n"
        "2. Import Jenga blocks into Isaac Sim, naming them the same as the real-world topics.\n"
        "3. Set pz (z-position) of each Jenga block to 0.\n"
        "4. Always use w, x, y, z orientation for importing.\n"
        "5. Jenga block asset path: omniverse://localhost/Library/Aruco/objs/jenga.usd"
    ),
    "custom_placement": (
        "## Custom Placement of Jenga Blocks in Isaac Sim\n"
        "Scene Description:\n"
        "{scene_description}\n"
        "Instructions:\n"
        "1. Import and place Jenga blocks in Isaac Sim according to the above scene description.\n"
        "2. Use Jenga blocks from: omniverse://localhost/Library/Aruco/objs/jenga.usd\n"
        "3. Ensure pz (z-position) of each Jenga block is set to 0."
    ),
    "reset_scene": (
        "## Reset Jenga Scene in Isaac Sim\n"
        "1. List available topics and read Jenga block poses (do not load scene).\n"
        "2. List prims in Isaac Sim and check if Jenga blocks are already imported.\n"
        "   - If a Jenga block exists in Isaac Sim but not in topics, delete that prim.\n"
        "   - If there are more Jenga blocks in topics than in the scene, add new Jenga blocks with the correct orientation.\n"
        "   - If a Jenga block exists in both, update its pose.\n"
        "3. Place all blocks in the scene in Isaac Sim.\n"
        "4. Jenga block asset path: omniverse://localhost/Library/Aruco/objs/jenga.usd"
    ),
    "hover": (
        "## Hover Over a Jenga Block\n"
        "Step 0: List available topics/prims and select one Jenga block.\n"
        "Step 1: Read the selected Jenga block's pose: x, y, z, rx, ry, rz.\n"
        "Step 2: Move UR end effector (ee) to x, y, (z=0.25) and set orientation to pick the Jenga block:\n"
        "        - Orientation: 0, 180, (UR rz), where UR rz = (90 - Jenga rz) × -1."
    ),
    "pick_and_place": (
        "## Pick & Place Operation for Jenga Blocks\n"
        "Step 0: List available topics/prims and select a Jenga block (not already moved).\n"
        "Step 1: Read the selected block's pose: x, y, z, rx, ry, rz.\n"
        "Step 2: Move UR end effector to x, y, (z=0.25) with orientation 0, 180, 0.\n"
        "Step 3: Read Jenga block pose again and calculate pick orientation:\n"
        "        - UR rz = (90 - Jenga rz) × -1\n"
        "        - Set ee orientation to 0, 180, (UR rz)\n"
        "Step 4: Move UR end effector to x, y, (z=0.15) with orientation 0, 180, (UR rz).\n"
        "Step 5: Close gripper to grasp the Jenga block.\n"
        "Step 6: ros2 topic echo /gripper_grasp_detected and check if its true. if not, send the open gripper once and then close gripper command till /gripper_grasp_detected says true.\n"
        "Step 7: Move UR end effector to x, y, (z=0.25).\n"
        "Step 7: Calculate and set end effector orientation for final placement:\n"
        "        - required_rotation = desired_final_rotation - current_jenga_rz\n"
        "        - new_ee_rz = current_ee_rz + required_rotation\n"
        "        - Set orientation to 0, 180, (new_ee_rz)\n"
        "Step 8: Move UR end effector to final pose: x, y, 0.25 (drop position).\n"
        "Step 9: Move end effector z to 0.151 for precise placement.\n"
        "Step 10: Open gripper to release the Jenga block.\n"
        "Step 11: Return to home position: HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0] if the sequence is complete. If there are more jenga blocks to pick go to pick station.\n"
        "\n"
        "Final drop positions to cycle through:\n"
        "{final_positions}\n"
        "\n"
        "Repeat the entire operation until no Jenga blocks remain in their initial pose.\n"
        "\n"
        "End Goal: {end_goal}"
     ),
     "recreate_dt": (
         "## Recreate DT (Digital Twin) Operation\n"
         "Step 1: Move UR end effector to picking station position [-0.361, -0.385, 0.481, 0, 180, 0] to scan Jenga blocks available for picking.\n"
         "Step 2: Get ROS2 topics and read topic info of one Jenga block to pick up from the real world.\n"
         "Step 3: Read the pose of the jenga block from json file attached \n"
         "Step 4: Assign available jenga blocks in the real world to the pose of jenga blocks in the json file.\n"
         "Step 5: Move Jenga block using pick and place prompt to replicate the position from json file in the real world.\n"
         "Step 6: Go back to picking station and repeat the process until all Jenga blocks in json have been replicated in the real world.\n"
         "End Goal: {end_goal}"
     ),
     "stack": (
         "## Stack Operation for Jenga Blocks\n"
         "Step 1: Move UR end effector to pick station position {pick_station} to scan Jenga blocks available for picking.\n"
         "Step 2: Get ROS2 topics and read topic info of one Jenga block to pick up from the real world.\n"
         "Step 3: use visual_servo_pick mcp tool to move ee to pick the jenga block.\n"
         "Step 4: Close gripper to grasp the Jenga block.\n"
         "Step 5: ros2 topic echo /gripper_grasp_detected and check if its true. if the grasp was not detected, Go 5 cm up to get updated position of jenga . Try sending the open gripper once and then close grippercommand till /gripper_grasp_detected says true.\n"         
         "Step 6: Move UR end effector to final pose: {final_x}, {final_y}, 0.25 (drop position), 0,180,0.\n"
         "Step 7: Run move_down mcp tool. \n"
         "Step 8: Return to pick station and repeat the process for all remaining Jenga blocks.\n"
         "Step 9 : move HOME_POSE = [0.065, -0.385, 0.481, 0, 180, 0] after there are no objects in the pick station \n"
     )
}


@mcp.prompt()
def load_jenga_block_in_sim(mode: str = "real_world", scene_description: str = None) -> str:
    """
    Returns a prompt for setting up and loading Jenga blocks in Isaac Sim.
    
    Args:
        mode: The loading mode - either "real_world" or "custom_placement"
        scene_description: Description of where to place the jenga blocks (required for custom_placement mode)
    
    Returns:
        str: The appropriate prompt for the specified mode
    """
    if mode == "custom_placement":
        if not scene_description:
            return "Error: scene_description is required for custom_placement mode"
        return PROMPTS["custom_placement"].format(scene_description=scene_description)
    return PROMPTS["real_world"]


@mcp.prompt()
def reset_jenga_scene() -> str:
    """
    Returns a prompt for resetting the Jenga blocks scene in Isaac Sim.
    This function will:
    1. List available topics and read jenga poses
    2. Check existing prims in Isaac Sim
    3. Update, delete, or add jenga blocks based on the available topics
    4. Ensure proper orientation and placement of all blocks
    
    Returns:
        str: The prompt for resetting the scene
    """
    return PROMPTS["reset_scene"]


@mcp.prompt()
def hover_over_jenga_block() -> str:
    """
    Returns a prompt for positioning the UR robot's end effector to hover over a jenga block.
    This function will:
    1close. List available topics/prims and select one jenga block
    2. Read the selected block's pose (position and orientation)
    3. Position the UR end effector above the block at z=0.25
    4. Calculate and apply the correct end effector orientation for picking
    
    Returns:
        str: The prompt for hovering over a jenga block
    """
    return PROMPTS["hover"]


@mcp.prompt()
def pick_and_place_jenga_blocks(final_positions: str = None, end_goal: str = None) -> str:
    """
    Returns a comprehensive prompt for performing pick and place operations on Jenga blocks.
    This function implements a complete workflow that:
    1. Identifies available Jenga blocks and selects one for manipulation
    2. Reads the block's current pose and calculates proper approach orientation
    3. Performs precise pick operation with gripper control
    4. Confirms successful grasp by ros2 topic echo /gripper_grasp_detected. if the grasp was not detected, Go 5 cm up to get updated position of jenga . Try sending the open gripper once and then close grippercommand till /gripper_grasp_detected says true.
    5. Transports the block to designated drop positions
    6. Places the block with proper orientation
    7. Returns to home position
    8. Repeats until all blocks are moved from initial to final positions
    
    Args:
        final_positions: Description of the final drop positions (e.g., "Position 1: [-0.25, -0.5, 0.151], Position 2: [-0.25, -0.41, 0.151], Position 3: [-0.25, -0.32, 0.151]")
        end_goal: Description of the end goal for this pick and place operation
    
    Returns:
        str: The comprehensive prompt for pick and place operations
    """
    # Default values if not provided
    # if final_positions is None:
    #     final_positions = (
    #         "- Position 1: [-0.25, -0.5, 0.151]\n"
    #         "- Position 2: [-0.25, -0.41, 0.151]\n"
    #         "- Position 3: [-0.25, -0.32, 0.151]"
    #     )
    
    # if end_goal is None:
    #     end_goal = "This pick and place operation systematically relocates all Jenga blocks from their initial scattered positions to three organized, aligned positions, creating a structured arrangement that demonstrates precise robotic manipulation capabilities while maintaining proper block orientation and ensuring successful grasp verification."
    
    return PROMPTS["pick_and_place"].format(
        final_positions=final_positions,
        end_goal=end_goal
    )


@mcp.prompt()
def recreate_dt(end_goal: str = None) -> str:
    """
    Returns a prompt for recreating the Digital Twin (DT) by replicating Isaac Sim scene in the real world.
    This function implements a workflow that:
    1. Moves to a designated picking station position for optimal scanning
    2. Reads real-world Jenga block poses from ROS2 topics
    3. Gets corresponding Isaac Sim block information
    4. Uses pick and place operations to replicate Isaac Sim positions in real world
    5. Repeats until all simulated blocks are replicated
    
    Args:
        end_goal: Description of the end goal for this DT recreation operation
    
    Returns:
        str: The prompt for DT recreation operations
    """
    # Default end goal if not provided
    if end_goal is None:
        end_goal = "Successfully replicate the complete Isaac Sim Jenga block arrangement in the real world, creating a perfect digital twin where the physical scene matches the simulated scene exactly, demonstrating the capability to bridge virtual and physical environments through precise robotic manipulation."
    
    return PROMPTS["recreate_dt"].format(end_goal=end_goal)


@mcp.prompt()
def stack_jenga_blocks(pick_station: str = None, final_x: float = None, final_y: float = None, jenga_orientation: float = None) -> str:
    """
    Returns a prompt for stacking Jenga blocks using force sensor feedback.
    This function implements a complete stacking workflow that:
    1. Moves to a designated pick station position for optimal scanning
    2. Reads real-world Jenga block poses from ROS2 topics
    3. Performs precise pick operations with gripper control
    4. Transports blocks to a specified stacking location
    5. Uses force sensor feedback to determine optimal placement height
    6. Stops downward movement when z force reaches -5.0 N
    7. Repeats until all blocks are stacked
    8. Places all blocks with the same specified orientation
    
    Args:
        pick_station: Pick station position [x, y, z, rx, ry, rz] (default: [-0.361, -0.385, 0.481, 0, 180, 0])
        final_x: X coordinate for the first Jenga block in the stack
        final_y: Y coordinate for the first Jenga block in the stack
        jenga_orientation: Target orientation in degrees for all Jenga blocks (default: 90.0)
    
    Returns:
        str: The prompt for stacking operations
    """
    # Default values if not provided
    if pick_station is None:
        pick_station = "[-0.361, -0.385, 0.481, 0, 180, 0]"
    
    if final_x is None:
        final_x = 0.0
    
    if final_y is None:
        final_y = -0.5
    
    if jenga_orientation is None:
        jenga_orientation = 90.0
    
    return PROMPTS["stack"].format(
        pick_station=pick_station,
        final_x=final_x,
        final_y=final_y,
        jenga_orientation=jenga_orientation
    )


if __name__ == "__main__":
    mcp.run()