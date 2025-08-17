from mcp import ClientSession, StdioServerParameters, types
from mcp.client.stdio import stdio_client
import asyncio
import traceback

server_params = StdioServerParameters(
    command = "uv",
    args = ["run", "weather.py"]
)

async def run():
    try:
        print("Starting MCP client...")
        async with stdio_client(server_params) as (read, write):
            print("Connected to MCP server")
            async with ClientSession(read, write) as session:
                print("Initialized client session")
                await session.initialize()
                print("listing tools")
                tools = await session.list_tools()
                print(f"Available tools: {tools}")

                print("Calling tool")
                result = await session.call_tool("get_weather", arguments={"city": "York"})
                print(f"Tool result: {result}")

    except Exception as e:
        print("an error occurred:")
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(run())