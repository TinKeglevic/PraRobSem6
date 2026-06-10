#!/usr/bin/env python3
"""Text-based ROSA interface. Interactive REPL with streaming output."""

import asyncio
import sys
import time

from rich.console import Console
from rich.panel import Panel

from prarob_interact.agent import create_agent
from prarob_interact.ros2_introspection import scan_ros2_environment

console = Console()


def _format_tool_args(event: dict) -> str:
    """Return a compact string of tool input arguments."""
    inputs = event.get("inputs", {})
    if not inputs:
        return ""
    parts = []
    for key, val in inputs.items():
        s = str(val)
        if len(s) > 80:
            s = s[:77] + "..."
        parts.append(f"{key}={s}")
    return " ".join(parts)


async def stream_response(agent, query: str):
    """Stream ROSA response tokens to the console."""
    full_response = ""
    tool_start_time = None
    try:
        async for event in agent.astream(query):
            kind = event.get("type", "")
            if kind == "token":
                token = event.get("content", "")
                console.print(token, end="", highlight=False)
                full_response += token
            elif kind == "tool_start":
                name = event.get("name", "tool")
                args = _format_tool_args(event)
                tool_start_time = time.monotonic()
                if args:
                    console.print(
                        f"\n  [bold cyan]{name}[/bold cyan] [dim]{args}[/dim]",
                    )
                else:
                    console.print(f"\n  [bold cyan]{name}[/bold cyan]")
            elif kind == "tool_end":
                elapsed = ""
                if tool_start_time is not None:
                    dt = time.monotonic() - tool_start_time
                    elapsed = f" [dim]({dt:.1f}s)[/dim]"
                    tool_start_time = None
                output = event.get("content", "")
                if output:
                    preview = output.strip().split("\n")[0]
                    if len(preview) > 100:
                        preview = preview[:97] + "..."
                    console.print(
                        f"  [green]\u2713[/green] {preview}{elapsed}"
                    )
                else:
                    console.print(f"  [green]\u2713 done[/green]{elapsed}")
            elif kind == "error":
                console.print(f"\n[red]Error: {event.get('content', '')}[/red]")
            elif kind == "final":
                final = event.get("content", "")
                if final and not full_response:
                    console.print(final, highlight=False)
                    full_response = final
    except Exception as e:
        console.print(f"\n[red]Stream error: {e}[/red]")
        # Fallback to non-streaming
        if not full_response:
            try:
                result = agent.invoke(query)
                console.print(result, highlight=False)
                full_response = result
            except Exception as e2:
                console.print(f"[red]Error: {e2}[/red]")

    if full_response:
        console.print()  # newline after streamed output


def run_sync(agent, query: str):
    """Fallback synchronous invoke."""
    try:
        result = agent.invoke(query)
        console.print(result, highlight=False)
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")


def main():
    console.print(
        Panel(
            "[bold]ROSA Text Interface[/bold]\n"
            "Type your queries below. Commands: [dim]clear, exit[/dim]",
            title="prarob_interact",
            border_style="blue",
        )
    )

    # Pre-scan ROS2 environment with visible progress
    console.print()
    try:
        ros2_state = scan_ros2_environment()
    except Exception as e:
        console.print(f"[yellow]ROS2 scan failed ({e}), continuing without pre-scan.[/yellow]")
        ros2_state = None

    console.print()

    try:
        agent = create_agent(streaming=True, ros2_state=ros2_state)
    except Exception as e:
        console.print(f"[red]Failed to create agent: {e}[/red]")
        sys.exit(1)

    console.print("[green]Agent ready.[/green]\n")

    while True:
        try:
            query = console.input("[bold blue]You>[/bold blue] ").strip()
        except (EOFError, KeyboardInterrupt):
            console.print("\n[dim]Goodbye.[/dim]")
            break

        if not query:
            continue
        if query.lower() in ("exit", "quit"):
            console.print("[dim]Goodbye.[/dim]")
            break
        if query.lower() == "clear":
            agent.clear_chat()
            console.print("[dim]Chat history cleared.[/dim]")
            continue

        console.print("[bold green]ROSA>[/bold green] ", end="")
        try:
            asyncio.run(stream_response(agent, query))
        except Exception:
            run_sync(agent, query)


if __name__ == "__main__":
    main()
