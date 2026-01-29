#!/usr/bin/env python3
"""
Report Graphs Generator

Generates matplotlib graphs for the support report PDF.
Uses Prime Inspections branding colors.
"""

import os
import tempfile
from datetime import datetime, timedelta

# Use non-interactive backend for server-side rendering
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.dates import DateFormatter
import numpy as np


# Prime Inspections brand colors
PRIME_CYAN = '#00C8D4'
PRIME_DARK = '#333333'
PRIME_GRAY = '#666666'
PRIME_LIGHT_GRAY = '#999999'
PRIME_GRID = '#E0E0E0'

# Status colors
COLOR_GREEN = '#00C851'
COLOR_YELLOW = '#FFD700'
COLOR_RED = '#FF4444'

# Motor line colors
MOTOR_0_COLOR = '#0066CC'  # Blue for Motor 0
MOTOR_1_COLOR = '#CC0000'  # Red for Motor 1


def setup_plot_style():
    """Apply Prime Inspections styling to matplotlib."""
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['Helvetica', 'Arial', 'DejaVu Sans'],
        'font.size': 10,
        'axes.titlesize': 12,
        'axes.titleweight': 'bold',
        'axes.labelsize': 10,
        'axes.labelcolor': PRIME_DARK,
        'axes.edgecolor': PRIME_GRAY,
        'axes.grid': True,
        'grid.color': PRIME_GRID,
        'grid.linestyle': '--',
        'grid.alpha': 0.5,
        'xtick.color': PRIME_DARK,
        'ytick.color': PRIME_DARK,
        'legend.fontsize': 9,
        'legend.framealpha': 0.9,
        'figure.facecolor': 'white',
        'axes.facecolor': 'white',
    })


def generate_velocity_graph(data, output_path=None):
    """
    Generate motor velocity over time graph.

    Args:
        data: List of dicts with 'timestamp', 'm0_velocity_in_s', 'm1_velocity_in_s'
        output_path: Path to save the image (if None, returns temp path)

    Returns:
        Path to the generated image
    """
    setup_plot_style()

    if not data:
        return None

    # Parse timestamps and values
    timestamps = []
    m0_vel = []
    m1_vel = []

    for row in data:
        try:
            if isinstance(row['timestamp'], str):
                ts = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S')
            else:
                ts = row['timestamp']
            timestamps.append(ts)
            m0_vel.append(row.get('m0_velocity_in_s', 0) or 0)
            m1_vel.append(row.get('m1_velocity_in_s', 0) or 0)
        except:
            continue

    if not timestamps:
        return None

    fig, ax = plt.subplots(figsize=(10, 3.5))

    # Plot data
    ax.plot(timestamps, m0_vel, color=MOTOR_0_COLOR, label='Motor 0 (Left)', linewidth=1.5)
    ax.plot(timestamps, m1_vel, color=MOTOR_1_COLOR, label='Motor 1 (Right)', linewidth=1.5, alpha=0.8)

    # Styling
    ax.set_xlabel('Time', fontsize=10, color=PRIME_DARK)
    ax.set_ylabel('Velocity (in/s)', fontsize=10, color=PRIME_DARK)
    ax.set_title('Motor Velocity - Past Hour', fontsize=12, fontweight='bold', color=PRIME_DARK)

    # Zero line
    ax.axhline(y=0, color=PRIME_GRAY, linestyle='-', linewidth=0.5)

    # Format x-axis
    ax.xaxis.set_major_formatter(DateFormatter('%H:%M'))
    plt.xticks(rotation=45)

    # Legend
    ax.legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()

    # Save
    if output_path is None:
        output_path = tempfile.mktemp(suffix='_velocity.png')

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()

    return output_path


def generate_torque_graph(data, output_path=None):
    """
    Generate motor torque over time graph with warning zones.

    Args:
        data: List of dicts with 'timestamp', 'm0_torque_nm', 'm1_torque_nm'
        output_path: Path to save the image (if None, returns temp path)

    Returns:
        Path to the generated image
    """
    setup_plot_style()

    if not data:
        return None

    # Parse timestamps and values
    timestamps = []
    m0_torque = []
    m1_torque = []

    for row in data:
        try:
            if isinstance(row['timestamp'], str):
                ts = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S')
            else:
                ts = row['timestamp']
            timestamps.append(ts)
            m0_torque.append(row.get('m0_torque_nm', 0) or 0)
            m1_torque.append(row.get('m1_torque_nm', 0) or 0)
        except:
            continue

    if not timestamps:
        return None

    fig, ax = plt.subplots(figsize=(10, 3.5))

    # Add color zones for torque levels
    # Green zone: 0 - 0.4 Nm (continuous rating)
    # Yellow zone: 0.4 - 1.6 Nm (intermittent)
    # Red zone: 1.6 - 2.0 Nm (peak)
    ax.axhspan(0, 0.4, color=COLOR_GREEN, alpha=0.1, label='Continuous')
    ax.axhspan(0.4, 1.6, color=COLOR_YELLOW, alpha=0.1, label='Intermittent')
    ax.axhspan(1.6, 2.0, color=COLOR_RED, alpha=0.1, label='Peak')

    # Continuous rating limit line
    ax.axhline(y=0.4, color=COLOR_YELLOW, linestyle='--', linewidth=1, label='Continuous Limit')

    # Plot data
    ax.plot(timestamps, m0_torque, color=MOTOR_0_COLOR, label='Motor 0', linewidth=1.5)
    ax.plot(timestamps, m1_torque, color=MOTOR_1_COLOR, label='Motor 1', linewidth=1.5, alpha=0.8)

    # Styling
    ax.set_xlabel('Time', fontsize=10, color=PRIME_DARK)
    ax.set_ylabel('Torque (Nm)', fontsize=10, color=PRIME_DARK)
    ax.set_title('Motor Torque - Past Hour', fontsize=12, fontweight='bold', color=PRIME_DARK)

    # Y-axis limits
    ax.set_ylim(0, 2.2)

    # Format x-axis
    ax.xaxis.set_major_formatter(DateFormatter('%H:%M'))
    plt.xticks(rotation=45)

    # Legend (outside plot area for clarity)
    ax.legend(loc='upper right', framealpha=0.9, fontsize=8)

    plt.tight_layout()

    # Save
    if output_path is None:
        output_path = tempfile.mktemp(suffix='_torque.png')

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()

    return output_path


def generate_temperature_graph(data, output_path=None):
    """
    Generate CPU temperature over time graph with warning thresholds.

    Args:
        data: List of dicts with 'timestamp', 'cpu_temp'
        output_path: Path to save the image (if None, returns temp path)

    Returns:
        Path to the generated image
    """
    setup_plot_style()

    if not data:
        return None

    # Parse timestamps and values
    timestamps = []
    temps = []

    for row in data:
        try:
            if isinstance(row['timestamp'], str):
                ts = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S')
            else:
                ts = row['timestamp']
            timestamps.append(ts)
            temps.append(row.get('cpu_temp', 0) or 0)
        except:
            continue

    if not timestamps:
        return None

    fig, ax = plt.subplots(figsize=(10, 3))

    # Warning threshold lines
    ax.axhline(y=70, color=COLOR_YELLOW, linestyle='--', linewidth=1, label='Warning (70C)')
    ax.axhline(y=80, color=COLOR_RED, linestyle='--', linewidth=1, label='Critical (80C)')

    # Plot temperature
    ax.plot(timestamps, temps, color=PRIME_CYAN, label='CPU Temp', linewidth=1.5)

    # Fill area above thresholds
    ax.fill_between(timestamps, temps, 70,
                    where=[t > 70 for t in temps],
                    color=COLOR_YELLOW, alpha=0.3)
    ax.fill_between(timestamps, temps, 80,
                    where=[t > 80 for t in temps],
                    color=COLOR_RED, alpha=0.3)

    # Styling
    ax.set_xlabel('Time', fontsize=10, color=PRIME_DARK)
    ax.set_ylabel('Temperature (C)', fontsize=10, color=PRIME_DARK)
    ax.set_title('CPU Temperature - Past Hour', fontsize=12, fontweight='bold', color=PRIME_DARK)

    # Y-axis range
    ax.set_ylim(30, 90)

    # Format x-axis
    ax.xaxis.set_major_formatter(DateFormatter('%H:%M'))
    plt.xticks(rotation=45)

    # Legend
    ax.legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()

    # Save
    if output_path is None:
        output_path = tempfile.mktemp(suffix='_temperature.png')

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()

    return output_path


def generate_position_graph(data, output_path=None):
    """
    Generate motor position over time graph.

    Args:
        data: List of dicts with 'timestamp', 'm0_position', 'm1_position'
        output_path: Path to save the image (if None, returns temp path)

    Returns:
        Path to the generated image
    """
    setup_plot_style()

    if not data:
        return None

    # Parse timestamps and values
    timestamps = []
    m0_pos = []
    m1_pos = []

    for row in data:
        try:
            if isinstance(row['timestamp'], str):
                ts = datetime.strptime(row['timestamp'], '%Y-%m-%d %H:%M:%S')
            else:
                ts = row['timestamp']
            timestamps.append(ts)
            m0_pos.append(row.get('m0_position', 0) or 0)
            m1_pos.append(row.get('m1_position', 0) or 0)
        except:
            continue

    if not timestamps:
        return None

    fig, ax = plt.subplots(figsize=(10, 3.5))

    # Plot data
    ax.plot(timestamps, m0_pos, color=MOTOR_0_COLOR, label='Motor 0 (Left)', linewidth=1.5)
    ax.plot(timestamps, m1_pos, color=MOTOR_1_COLOR, label='Motor 1 (Right)', linewidth=1.5, alpha=0.8)

    # Styling
    ax.set_xlabel('Time', fontsize=10, color=PRIME_DARK)
    ax.set_ylabel('Position (steps)', fontsize=10, color=PRIME_DARK)
    ax.set_title('Motor Position - Past Hour', fontsize=12, fontweight='bold', color=PRIME_DARK)

    # Format y-axis with comma separators for large numbers
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, p: format(int(x), ',')))

    # Format x-axis
    ax.xaxis.set_major_formatter(DateFormatter('%H:%M'))
    plt.xticks(rotation=45)

    # Legend
    ax.legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()

    # Save
    if output_path is None:
        output_path = tempfile.mktemp(suffix='_position.png')

    plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close()

    return output_path


def generate_all_graphs(data):
    """
    Generate all motor graphs and return paths.

    Args:
        data: List of dicts from motor_data_logger database

    Returns:
        Dict with paths to generated images
    """
    return {
        'velocity': generate_velocity_graph(data),
        'torque': generate_torque_graph(data),
        'temperature': generate_temperature_graph(data),
        'position': generate_position_graph(data)
    }


def cleanup_graph_files(graph_paths):
    """Remove temporary graph files."""
    for path in graph_paths.values():
        if path and os.path.exists(path):
            try:
                os.remove(path)
            except:
                pass


if __name__ == '__main__':
    # Test with sample data
    from datetime import datetime, timedelta
    import random

    now = datetime.now()
    test_data = []

    for i in range(60):  # 60 data points
        ts = now - timedelta(minutes=60-i)
        test_data.append({
            'timestamp': ts,
            'm0_velocity_in_s': random.uniform(0, 0.5) if i % 10 < 7 else 0,
            'm1_velocity_in_s': random.uniform(0, 0.5) if i % 10 < 7 else 0,
            'm0_torque_nm': random.uniform(0.05, 0.3),
            'm1_torque_nm': random.uniform(0.05, 0.3),
            'm0_position': i * 1000 + random.randint(-100, 100),
            'm1_position': i * 1000 + random.randint(-100, 100),
            'cpu_temp': 45 + random.uniform(-5, 15)
        })

    graphs = generate_all_graphs(test_data)
    print("Generated graphs:")
    for name, path in graphs.items():
        print(f"  {name}: {path}")
