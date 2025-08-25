# to use it with claude, run the following command : uv run server.py.

from mcp.server.fastmcp import FastMCP

mcp = FastMCP("server")

@mcp.tool()
def greeting(name: str) -> str:
    "Send a greeting"
    return f"Hello {name}!"

if __name__ == "__main__":
    mcp.run(transport="streamable-http")