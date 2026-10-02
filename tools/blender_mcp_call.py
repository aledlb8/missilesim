"""Call the installed local Blender MCP over stdio; Blender add-on must be running.

python tools/blender_mcp_call.py get_scene_info
python tools/blender_mcp_call.py execute_blender_code --code-file tools/build_rafale.py
Requires the MCP Python SDK and an installed/cached blender-mcp via uv.
"""
import argparse
import asyncio
import base64
import json
from pathlib import Path
import shutil

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

PROMPT = ('Using the blender MCP create the Rafale model so we can test that '
          'everything looks good and works for the game. If you need more '
          "information that isn't in the modeling markdown files research, you "
          "can search yourself, but if it's there you start working.")


async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tool')
    parser.add_argument('--code-file')
    parser.add_argument('--arguments', default='{}')
    parser.add_argument('--image-output')
    args = parser.parse_args()
    params = json.loads(args.arguments)
    params['user_prompt'] = PROMPT
    if args.code_file:
        source = Path(args.code_file).resolve()
        params['code'] = '__file__ = ' + repr(str(source)) + '\n' + source.read_text(encoding='utf-8')
    server = StdioServerParameters(command=shutil.which('uv'),
                                  args=['tool', 'run', '--offline', 'blender-mcp'])
    async with stdio_client(server) as (read, write):
        async with ClientSession(read, write, read_timeout_seconds=240) as session:
            await session.initialize()
            result = await session.call_tool(args.tool, params)
            for block in result.content:
                if block.type == 'text':
                    print(block.text)
                    if block.text.startswith(('Error executing code:', 'Rejected by safe mode')):
                        raise RuntimeError('Blender MCP execution failed; inspect the response above')
                elif block.type == 'image' and args.image_output:
                    Path(args.image_output).write_bytes(base64.b64decode(block.data))
                    print('Image saved:', args.image_output)
            if getattr(result, 'is_error', getattr(result, 'isError', False)):
                raise RuntimeError('Blender MCP tool failed')


if __name__ == '__main__':
    asyncio.run(main())
