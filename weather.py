from mcp.server.fastmcp import FastMCP

mcp = FastMCP()

@mcp.tool()
def get_weather(city: str) -> str: 
    """
    Get the weather for a given city
    Args:
        city: The city to get the weather for
    Returns:
        The weather for the given city
    """
    return f"The weather in {city} is sunny"

if __name__ == "__main__":
    mcp.run()