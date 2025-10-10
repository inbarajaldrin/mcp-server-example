from mcp.server.fastmcp import FastMCP
import json
from pathlib import Path

mcp = FastMCP("Assembly Poses Management")

# Path to persistent assembly poses data files
FINAL_ASSEMBLY_POSES_FILE = Path(__file__).parent / "data" / "final_assembly_poses.json"
INITIAL_OBJECT_POSES_FILE = Path(__file__).parent / "data" / "initial_object_poses.json"
LLM_DT_ANALYSIS_FILE = Path(__file__).parent / "data" / "llm_dt_analysis.json"

@mcp.tool()
def read_final_assembly_poses_file() -> str:
    """
    Read the final assembly poses file directly and return its raw content
    
    Returns:
        JSON string containing the raw file content
    """
    try:
        with open(FINAL_ASSEMBLY_POSES_FILE, 'r') as f:
            content = f.read()
        return content
    except FileNotFoundError:
        return json.dumps({
            "error": f"Assembly poses file not found at {FINAL_ASSEMBLY_POSES_FILE}",
            "message": "Please ensure data/final_assembly_poses.json exists."
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "error": f"Failed to read file: {str(e)}"
        }, indent=2)

@mcp.tool()
def update_object_pose_complete(pose_data: str) -> str:
    """
    Update assembly poses by providing complete pose data as a JSON string.
    This tool will overwrite the entire file with the new data structure.
    
    Expected JSON format:
    {
        "objects": [
            {
                "name": "Object1",
                "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                "rotation": {"x": 0.0, "y": 90.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707}
            }
        ]
    }
    
    Args:
        pose_data: JSON string containing the complete pose data structure
    
    Returns:
        JSON string with confirmation or error message
    """
    try:
        # Ensure data directory exists
        FINAL_ASSEMBLY_POSES_FILE.parent.mkdir(parents=True, exist_ok=True)
        
        # Parse the input JSON string
        new_data = json.loads(pose_data)
        
        # Validate the structure has "objects" key
        if "objects" not in new_data:
            return json.dumps({
                "success": False,
                "error": "Invalid data structure. Must contain 'objects' key.",
                "expected_format": {
                    "objects": [
                        {
                            "name": "Object1",
                            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                            "rotation": {"x": 0.0, "y": 90.0, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707}
                        }
                    ]
                }
            }, indent=2)
        
        # Overwrite the entire file with the new data
        with open(FINAL_ASSEMBLY_POSES_FILE, 'w') as f:
            json.dump(new_data, f, indent=2)
        
        return json.dumps({
            "success": True,
            "message": "Updated assembly poses and overwrote file",
            "objects_count": len(new_data["objects"]),
            "file_overwritten": True
        }, indent=2)
        
    except json.JSONDecodeError as e:
        return json.dumps({
            "success": False,
            "error": f"Invalid JSON format: {str(e)}"
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "success": False,
            "error": f"Failed to update file: {str(e)}"
        }, indent=2)

@mcp.tool()
def read_initial_object_poses_file() -> str:
    """
    Read the initial object poses file directly and return its raw content
    
    Returns:
        JSON string containing the raw file content
    """
    try:
        with open(INITIAL_OBJECT_POSES_FILE, 'r') as f:
            content = f.read()
        return content
    except FileNotFoundError:
        return json.dumps({
            "error": f"Initial object poses file not found at {INITIAL_OBJECT_POSES_FILE}",
            "message": "Please ensure data/initial_object_poses.json exists."
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "error": f"Failed to read file: {str(e)}"
        }, indent=2)

@mcp.tool()
def update_initial_object_pose_complete(pose_data: str) -> str:
    """
    Update initial object poses by providing complete pose data as a JSON string.
    This tool will overwrite the entire file with the new data structure.
    
    Expected JSON format:
    {
        "objects": [
            {
                "name": "Object1",
                "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                "rotation": {"x": 0.0, "y": 90.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707}
            }
        ]
    }
    
    Args:
        pose_data: JSON string containing the complete pose data structure
    
    Returns:
        JSON string with confirmation or error message
    """
    try:
        # Ensure data directory exists
        INITIAL_OBJECT_POSES_FILE.parent.mkdir(parents=True, exist_ok=True)
        
        # Parse the input JSON string
        new_data = json.loads(pose_data)
        
        # Validate the structure has "objects" key
        if "objects" not in new_data:
            return json.dumps({
                "success": False,
                "error": "Invalid data structure. Must contain 'objects' key.",
                "expected_format": {
                    "objects": [
                        {
                            "name": "Object1",
                            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                            "rotation": {"x": 0.0, "y": 90.0, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707}
                        }
                    ]
                }
            }, indent=2)
        
        # Overwrite the entire file with the new data
        with open(INITIAL_OBJECT_POSES_FILE, 'w') as f:
            json.dump(new_data, f, indent=2)
        
        return json.dumps({
            "success": True,
            "message": "Updated initial object poses and overwrote file",
            "objects_count": len(new_data["objects"]),
            "file_overwritten": True
        }, indent=2)
        
    except json.JSONDecodeError as e:
        return json.dumps({
            "success": False,
            "error": f"Invalid JSON format: {str(e)}"
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "success": False,
            "error": f"Failed to update file: {str(e)}"
        }, indent=2)

@mcp.tool()
def read_llm_dt_analysis_file() -> str:
    """
    Read the LLM DT analysis file directly and return its raw content
    
    Returns:
        JSON string containing the raw file content
    """
    try:
        with open(LLM_DT_ANALYSIS_FILE, 'r') as f:
            content = f.read()
        return content
    except FileNotFoundError:
        return json.dumps({
            "error": f"LLM DT analysis file not found at {LLM_DT_ANALYSIS_FILE}",
            "message": "Please ensure data/llm_dt_analysis.json exists."
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "error": f"Failed to read file: {str(e)}"
        }, indent=2)

@mcp.tool()
def update_llm_dt_analysis_complete(analysis_data: str) -> str:
    """
    Update LLM DT analysis by providing complete analysis data as a JSON string.
    This tool will overwrite the entire file with the new data structure.

    Expected JSON format:
    [
        {
            "block": "jenga_2",
            "operation": "pick_and_place",
            "target_position": {
                "x": 0.0,
                "y": -0.5,
                "z": 0.007
            }
        },
        {
            "block": "jenga_4",
            "operation": "push",
            "target_position": {
                "x": 0.024,
                "y": -0.5,
                "z": 0.007
            },
            "yaw": 0.0
        }
    ]
    
    Args:
        analysis_data: JSON string containing the complete analysis data structure
    
    Returns:
        JSON string with confirmation or error message
    """
    try:
        # Ensure data directory exists
        LLM_DT_ANALYSIS_FILE.parent.mkdir(parents=True, exist_ok=True)
        
        # Parse the input JSON string
        new_data = json.loads(analysis_data)
        
        # Validate the structure is a list
        if not isinstance(new_data, list):
            return json.dumps({
                "success": False,
                "error": "Invalid data structure. Must be a list of analysis entries.",
                "expected_format": [
                    {
                        "block": "jenga_2",
                        "operation": "pick_and_place",
                        "target_position": {
                            "x": 0.0,
                            "y": -0.5,
                            "z": 0.007
                        }
                    }
                ]
            }, indent=2)
        
        # Overwrite the entire file with the new data
        with open(LLM_DT_ANALYSIS_FILE, 'w') as f:
            json.dump(new_data, f, indent=2)
        
        return json.dumps({
            "success": True,
            "message": "Updated LLM DT analysis and overwrote file",
            "entries_count": len(new_data),
            "file_overwritten": True
        }, indent=2)
        
    except json.JSONDecodeError as e:
        return json.dumps({
            "success": False,
            "error": f"Invalid JSON format: {str(e)}"
        }, indent=2)
    except Exception as e:
        return json.dumps({
            "success": False,
            "error": f"Failed to update file: {str(e)}"
        }, indent=2)


if __name__ == "__main__":
    mcp.run()
