import streamlit as st
import asyncio
import json
import traceback
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
from openai import OpenAI
import os
from dotenv import load_dotenv
import sys

# Load MCP configuration from config file
def load_mcp_config(config_path="mcp_config.json"):
    """Load MCP server configuration from JSON file"""
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        return config
    except FileNotFoundError:
        print(f"Config file {config_path} not found")
        return None
    except json.JSONDecodeError as e:
        print(f"Error parsing config file: {e}")
        return None

# Load environment variables
load_dotenv()

# Load configuration
config = load_mcp_config()
if not config:
    print("Failed to load MCP configuration")
    exit(1)

# Find all enabled servers
enabled_servers = []
for server_name, server_config in config["mcpServers"].items():
    if not server_config.get("disabled", False):
        enabled_servers.append((server_name, server_config))

if not enabled_servers:
    print("No enabled servers found in configuration")
    exit(1)

# Store server configurations for UI selection
SERVER_CONFIGS = {name: config for name, config in enabled_servers}

def get_openai_tools(tools_result, enabled_tool_names=None):
    """Convert MCP tools to OpenAI format, optionally filtering by enabled tools"""
    all_tools = [
        {
            "type": "function",
            "function": {
                "name": tool.name,
                "description": tool.description,
                "parameters": tool.inputSchema,
            },
        }
        for tool in tools_result.tools
    ]
    
    # If no filter is provided, return all tools
    if enabled_tool_names is None:
        return all_tools
    
    # Filter tools based on enabled list
    return [tool for tool in all_tools if tool["function"]["name"] in enabled_tool_names]

def fetch_available_tools(server_name):
    """Fetch available tools from a server synchronously"""
    async def _fetch():
        server_config = SERVER_CONFIGS[server_name]
        server_params = StdioServerParameters(
            command=server_config["command"],
            args=server_config["args"]
        )
        
        try:
            async with stdio_client(server_params) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    tools_result = await session.list_tools()
                    return [(tool.name, tool.description) for tool in tools_result.tools]
        except Exception as e:
            st.error(f"Error fetching tools from {server_name}: {str(e)}")
            return []
    
    return asyncio.run(_fetch())

async def chat_response(messages, session, openai_tools, client):
    response = client.chat.completions.create(
        model='gpt-4o',
        messages=messages,
        tools=openai_tools,
        tool_choice="auto",
    )
    messages.append(response.choices[0].message)
    tool_calls = response.choices[0].message.tool_calls
    # Handle any tool calls
    if response.choices[0].message.tool_calls:
        for tool_execution in tool_calls:
            # Execute tool call
            result = await session.call_tool(
                tool_execution.function.name,
                arguments=json.loads(tool_execution.function.arguments),
            )
            # Add tool response to conversation
            messages.append(
                {
                    "role": "tool",
                    "tool_call_id": tool_execution.id,
                    "content": result.content[0].text,
                }
            )
            # Get response from LLM
            response = client.chat.completions.create(
                model='gpt-4o',
                messages=messages,
                tools=openai_tools,
                tool_choice="auto",
            )
            if response.choices[0].finish_reason == "tool_calls":
                tool_calls.extend(response.choices[0].message.tool_calls)
            if response.choices[0].finish_reason == "stop":
                messages.append(response.choices[0].message)
                return response.choices[0].message.content
    else:
        messages.append(response.choices[0].message)
        return response.choices[0].message.content

def sync_chat_response(messages, user_input, server_name, enabled_tools=None):
    async def _chat():
        # Get server configuration for the selected server
        server_config = SERVER_CONFIGS[server_name]
        server_params = StdioServerParameters(
            command=server_config["command"],
            args=server_config["args"]
        )
        
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                tools_result = await session.list_tools()
                openai_tools = get_openai_tools(tools_result, enabled_tools)
                client = OpenAI()
                messages.append({"role": "user", "content": user_input})
                response = await chat_response(messages, session, openai_tools, client)
                return response
    return asyncio.run(_chat())

def main():
    st.title("MCP Chatbot (Multi-Server Support)")
    
    # Server selection
    server_names = list(SERVER_CONFIGS.keys())
    selected_server = st.selectbox(
        "Select MCP Server:",
        server_names,
        key="server_selection"
    )
    
    st.info(f"Selected server: **{selected_server}**")
    
    # Initialize session state
    if "messages" not in st.session_state:
        st.session_state["messages"] = []
    if "history" not in st.session_state:
        st.session_state["history"] = []
    if "current_server" not in st.session_state:
        st.session_state["current_server"] = selected_server
    if "available_tools" not in st.session_state:
        st.session_state["available_tools"] = {}
    if "enabled_tools" not in st.session_state:
        st.session_state["enabled_tools"] = {}
    
    # Reset chat if server changes
    if st.session_state["current_server"] != selected_server:
        st.session_state["messages"] = []
        st.session_state["history"] = []
        st.session_state["current_server"] = selected_server
        st.rerun()
    
    # Fetch available tools for the selected server (cached per server)
    if selected_server not in st.session_state["available_tools"]:
        with st.spinner(f"Fetching tools from {selected_server}..."):
            tools = fetch_available_tools(selected_server)
            st.session_state["available_tools"][selected_server] = tools
            # By default, enable all tools
            st.session_state["enabled_tools"][selected_server] = [tool[0] for tool in tools]
    
    # Tool selection in sidebar
    with st.sidebar:
        st.header("🛠️ Tool Settings")
        st.subheader(f"Tools for: {selected_server}")
        
        available_tools = st.session_state["available_tools"].get(selected_server, [])
        
        if available_tools:
            # Add "Select All" / "Deselect All" buttons
            col1, col2 = st.columns(2)
            with col1:
                if st.button("✓ Select All"):
                    st.session_state["enabled_tools"][selected_server] = [tool[0] for tool in available_tools]
                    st.rerun()
            with col2:
                if st.button("✗ Deselect All"):
                    st.session_state["enabled_tools"][selected_server] = []
                    st.rerun()
            
            st.markdown("---")
            st.caption(f"**{len(available_tools)} tools available**")
            
            # Show enabled tool count
            enabled_count = len(st.session_state["enabled_tools"].get(selected_server, []))
            st.info(f"✓ {enabled_count} / {len(available_tools)} tools enabled")
            
            st.markdown("---")
            
            # Display each tool with checkbox
            for tool_name, tool_description in available_tools:
                is_enabled = tool_name in st.session_state["enabled_tools"].get(selected_server, [])
                
                # Create checkbox for each tool
                enabled = st.checkbox(
                    f"**{tool_name}**",
                    value=is_enabled,
                    key=f"tool_{selected_server}_{tool_name}",
                    help=tool_description[:200] + "..." if len(tool_description) > 200 else tool_description
                )
                
                # Update enabled tools list
                if enabled and tool_name not in st.session_state["enabled_tools"][selected_server]:
                    st.session_state["enabled_tools"][selected_server].append(tool_name)
                elif not enabled and tool_name in st.session_state["enabled_tools"][selected_server]:
                    st.session_state["enabled_tools"][selected_server].remove(tool_name)
        else:
            st.warning("No tools available for this server")
    
    # Main chat interface
    reset = st.button("Reset Chat")
    if reset:
        st.session_state["messages"] = []
        st.session_state["history"] = []
        st.rerun()
    
    user_input = st.text_input("You:", key="user_input")
    
    # Get enabled tools for the selected server
    enabled_tools = st.session_state["enabled_tools"].get(selected_server, None)
    
    # Warn if no tools are enabled
    if enabled_tools is not None and len(enabled_tools) == 0:
        st.warning("⚠️ No tools are enabled. The AI will not be able to use any server functions.")
    
    if st.button("Send") and user_input:
        with st.spinner(f"Thinking with {selected_server}..."):
            try:
                response = sync_chat_response(
                    st.session_state["messages"], 
                    user_input, 
                    selected_server,
                    enabled_tools if enabled_tools else None
                )
                st.session_state["history"].append((user_input, response))
                st.rerun()
            except Exception as e:
                st.error("An error occurred:")
                st.error(traceback.format_exc())
    
    # Display chat history
    for user, bot in st.session_state["history"]:
        st.markdown(f"**You:** {user}")
        st.markdown(f"**AI:** {bot}")

def run_streamlit():
    main()

if __name__ == "__main__":
    run_streamlit()
