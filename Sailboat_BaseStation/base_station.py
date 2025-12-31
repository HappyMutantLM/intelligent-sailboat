#!/usr/bin/env python3
"""
Intelligent Sailboat - Base Station GUI
Raspberry Pi 4 with 7" Touchscreen
Communicates with boat via Feather M0 LoRa USB dongle

Author: Leila
Date: 2024-12-30
"""

import tkinter as tk
from tkinter import ttk, messagebox
import tkintermapview
import serial
import struct
import threading
import time
import json
from pathlib import Path
from dataclasses import dataclass
from typing import Optional
import queue

# === CONFIGURATION MANAGER ===

class Config:
    """Configuration manager for base station"""
    
    DEFAULT_CONFIG_PATH = Path(__file__).parent / "config.json"
    
    def __init__(self, config_path=None):
        self.config_path = config_path or self.DEFAULT_CONFIG_PATH
        self.data = self.load_config()
    
    def load_config(self):
        """Load configuration from JSON file"""
        try:
            with open(self.config_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"⚠️  Config file not found: {self.config_path}")
            print("Creating default config...")
            defaults = self.get_defaults()
            self.data = defaults
            self.save_config()
            return defaults
        except json.JSONDecodeError as e:
            print(f"⚠️  Config file parse error: {e}")
            print("Using default values...")
            return self.get_defaults()
    
    def save_config(self):
        """Save current configuration to file"""
        try:
            with open(self.config_path, 'w') as f:
                json.dump(self.data, f, indent=2)
            print(f"✅ Config saved to {self.config_path}")
        except Exception as e:
            print(f"❌ Failed to save config: {e}")
    
    def get(self, *keys, default=None):
        """Get nested config value: config.get('serial', 'port')"""
        value = self.data
        for key in keys:
            if isinstance(value, dict):
                value = value.get(key)
            else:
                return default
        return value if value is not None else default
    
    def set(self, *keys, value):
        """Set nested config value: config.set('serial', 'port', value='/dev/ttyACM1')"""
        if len(keys) < 1:
            return
        
        # Navigate to parent dict
        current = self.data
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        # Set final value
        current[keys[-1]] = value
    
    @staticmethod
    def get_defaults():
        """Fallback default configuration"""
        return {
            "serial": {
                "port": "/dev/ttyACM0",
                "baud_rate": 115200,
                "timeout": 0.1,
                "reconnect_attempts": 3,
                "reconnect_delay": 2.0
            },
            "map": {
                "default_lat": 35.7796,
                "default_lon": -78.6382,
                "default_zoom": 15,
                "track_max_points": 500,
                "track_color": "blue",
                "track_width": 3,
                "auto_center_on_boat": True,
                "waypoint_marker_color": "red"
            },
            "telemetry": {
                "update_rate_hz": 10,
                "stale_data_timeout_sec": 5.0,
                "battery_warn_voltage": 7.0,
                "battery_critical_voltage": 6.5,
                "rssi_warn_threshold": -100,
                "rssi_critical_threshold": -110
            },
            "ui": {
                "fullscreen": False,
                "window_width": 1280,
                "window_height": 720,
                "theme": "dark"
            },
            "navigation": {
                "waypoint_arrival_radius_m": 10.0,
                "default_sail_trim": 50,
                "default_rudder_center": 0
            },
            "logging": {
                "enable_data_logging": True,
                "log_directory": "./logs"
            },
            "advanced": {
                "protocol_version": "1.0.0",
                "checksum_validation": True
            }
        }

# === PROTOCOL DEFINITIONS ===

# Packet type constants (match Protocol.h)
PKT_CONTROL = 0x01
PKT_TELEMETRY = 0x02
PKT_WAYPOINT = 0x03
PKT_MODE_CHANGE = 0x04

# Mode constants
MODE_MANUAL = 0x00
MODE_AUTONOMOUS = 0x01

@dataclass
class TelemetryData:
    """Parsed telemetry from boat"""
    lat: float = 0.0
    lon: float = 0.0
    heading: float = 0.0
    wind_angle: float = 0.0
    battery: float = 0.0
    rssi: int = -999
    timestamp: float = 0.0

class ProtocolHandler:
    """Handle binary protocol packing/unpacking"""
    
    @staticmethod
    def pack_waypoint(lat: float, lon: float) -> bytes:
        """Create waypoint packet"""
        packet = struct.pack('<Bfff', PKT_WAYPOINT, lat, lon, 0)
        checksum = ProtocolHandler._calculate_checksum(packet[:-1])
        return packet[:-1] + struct.pack('B', checksum)
    
    @staticmethod
    def pack_mode(mode: int) -> bytes:
        """Create mode change packet"""
        packet = struct.pack('<BBB', PKT_MODE_CHANGE, mode, 0)
        checksum = ProtocolHandler._calculate_checksum(packet[:-1])
        return packet[:-1] + struct.pack('B', checksum)
    
    @staticmethod
    def pack_control(rudder: int, sail: int) -> bytes:
        """Create manual control packet"""
        packet = struct.pack('<Bhhb', PKT_CONTROL, rudder, sail, 0)
        checksum = ProtocolHandler._calculate_checksum(packet[:-1])
        return packet[:-1] + struct.pack('B', checksum)
    
    @staticmethod
    def unpack_telemetry(data: bytes) -> Optional[TelemetryData]:
        """Parse telemetry packet"""
        if len(data) < 22:
            return None
        
        try:
            pkt_type, lat, lon, heading, wind, battery, checksum = struct.unpack('<Bfffffb', data[:22])
            
            if pkt_type != PKT_TELEMETRY:
                return None
            
            # Verify checksum
            calc_checksum = ProtocolHandler._calculate_checksum(data[:21])
            if calc_checksum != checksum:
                print(f"Checksum mismatch! Expected {calc_checksum}, got {checksum}")
                return None
            
            return TelemetryData(
                lat=lat,
                lon=lon,
                heading=heading,
                wind_angle=wind,
                battery=battery,
                timestamp=time.time()
            )
        except struct.error as e:
            print(f"Unpack error: {e}")
            return None
    
    @staticmethod
    def _calculate_checksum(data: bytes) -> int:
        """Simple XOR checksum"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

# === MAIN APPLICATION ===

class MissionControlApp:
    def __init__(self, root):
        self.root = root
        
        # Load configuration
        self.config = Config()
        
        self.root.title("Intelligent Sailboat - Base Station")
        
        # Apply UI config
        if self.config.get('ui', 'fullscreen', default=False):
            self.root.attributes('-fullscreen', True)
        else:
            width = self.config.get('ui', 'window_width', default=1280)
            height = self.config.get('ui', 'window_height', default=720)
            self.root.geometry(f"{width}x{height}")
        
        # Serial configuration
        self.serial_port = self.config.get('serial', 'port', default='/dev/ttyACM0')
        self.baud_rate = self.config.get('serial', 'baud_rate', default=115200)
        self.serial_timeout = self.config.get('serial', 'timeout', default=0.1)
        
        # State
        self.ser: Optional[serial.Serial] = None
        self.is_connected = False
        self.telemetry_queue = queue.Queue()
        self.track_points = []
        
        # Track configuration
        self.track_max_points = self.config.get('map', 'track_max_points', default=500)
        self.track_color = self.config.get('map', 'track_color', default='blue')
        self.track_width = self.config.get('map', 'track_width', default=3)
        
        # Telemetry configuration
        self.update_rate_ms = int(1000 / self.config.get('telemetry', 'update_rate_hz', default=10))
        self.battery_warn = self.config.get('telemetry', 'battery_warn_voltage', default=7.0)
        self.battery_crit = self.config.get('telemetry', 'battery_critical_voltage', default=6.5)
        
        # Protocol handler
        self.protocol = ProtocolHandler()
        
        # Current boat state
        self.current_telemetry = TelemetryData()
        self.current_mode = MODE_MANUAL
        
        # === BUILD UI ===
        self.build_ui()
        
        # Start UI update loop
        self.update_ui()
    
    def build_ui(self):
        """Construct the GUI"""
        
        # === TOP BAR ===
        top_frame = tk.Frame(self.root, bg="#2c3e50", height=60)
        top_frame.pack(fill="x", side="top")
        top_frame.pack_propagate(False)
        
        # Connection button
        self.btn_connect = tk.Button(
            top_frame, 
            text="⚡ CONNECT", 
            command=self.toggle_connection,
            font=("Arial", 14, "bold"),
            bg="#27ae60",
            fg="white",
            activebackground="#229954",
            relief="flat",
            padx=20
        )
        self.btn_connect.pack(side="left", padx=10, pady=10)
        
        # Status label
        self.lbl_status = tk.Label(
            top_frame,
            text="DISCONNECTED",
            font=("Arial", 14, "bold"),
            bg="#2c3e50",
            fg="#e74c3c"
        )
        self.lbl_status.pack(side="left", padx=20)
        
        # Settings button
        tk.Button(
            top_frame,
            text="⚙️",
            command=self.show_settings_dialog,
            font=("Arial", 14),
            bg="#95a5a6",
            fg="white",
            relief="flat",
            padx=15
        ).pack(side="right", padx=10, pady=10)
        
        # Mode indicator
        self.lbl_mode = tk.Label(
            top_frame,
            text="MODE: MANUAL",
            font=("Arial", 14, "bold"),
            bg="#2c3e50",
            fg="#f39c12"
        )
        self.lbl_mode.pack(side="right", padx=20)
        
        # === MAIN CONTENT (Left/Right Split) ===
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill="both", expand=True)
        
        # Left panel (controls + telemetry)
        left_panel = tk.Frame(main_frame, width=400, bg="#34495e")
        left_panel.pack(side="left", fill="y")
        left_panel.pack_propagate(False)
        
        # Right panel (map)
        right_panel = tk.Frame(main_frame, bg="black")
        right_panel.pack(side="right", fill="both", expand=True)
        
        # === LEFT PANEL CONTENT ===
        self.build_left_panel(left_panel)
        
        # === RIGHT PANEL (MAP) ===
        self.build_map(right_panel)
    
    def build_left_panel(self, parent):
        """Build telemetry and controls"""
        
        # Telemetry section
        telem_frame = tk.LabelFrame(
            parent,
            text="📡 LIVE TELEMETRY",
            font=("Arial", 12, "bold"),
            bg="#34495e",
            fg="white",
            padx=10,
            pady=10
        )
        telem_frame.pack(fill="x", padx=10, pady=10)
        
        # Telemetry labels
        label_style = {"font": ("Courier", 11), "bg": "#34495e", "fg": "#ecf0f1", "anchor": "w"}
        
        self.lbl_position = tk.Label(telem_frame, text="GPS: --°, --°", **label_style)
        self.lbl_position.pack(fill="x", pady=2)
        
        self.lbl_heading = tk.Label(telem_frame, text="HDG: ---°", **label_style)
        self.lbl_heading.pack(fill="x", pady=2)
        
        self.lbl_wind = tk.Label(telem_frame, text="WIND: ---° (rel)", **label_style)
        self.lbl_wind.pack(fill="x", pady=2)
        
        self.lbl_battery = tk.Label(telem_frame, text="BAT: --.-- V", **label_style)
        self.lbl_battery.pack(fill="x", pady=2)
        
        self.lbl_rssi = tk.Label(telem_frame, text="RSSI: --- dBm", **label_style)
        self.lbl_rssi.pack(fill="x", pady=2)
        
        # Navigation section
        nav_frame = tk.LabelFrame(
            parent,
            text="🎯 WAYPOINT",
            font=("Arial", 12, "bold"),
            bg="#34495e",
            fg="white",
            padx=10,
            pady=10
        )
        nav_frame.pack(fill="x", padx=10, pady=10)
        
        tk.Label(nav_frame, text="Tap map to set destination", bg="#34495e", fg="#bdc3c7").pack()
        
        btn_autonomous = tk.Button(
            nav_frame,
            text="🚤 ENABLE AUTONOMOUS",
            command=self.enable_autonomous_mode,
            font=("Arial", 11, "bold"),
            bg="#3498db",
            fg="white",
            relief="flat",
            padx=10,
            pady=8
        )
        btn_autonomous.pack(fill="x", pady=5)
        
        btn_manual = tk.Button(
            nav_frame,
            text="🎮 MANUAL MODE",
            command=self.enable_manual_mode,
            font=("Arial", 11, "bold"),
            bg="#95a5a6",
            fg="white",
            relief="flat",
            padx=10,
            pady=8
        )
        btn_manual.pack(fill="x", pady=5)
        
        # Manual control section
        control_frame = tk.LabelFrame(
            parent,
            text="🎮 MANUAL CONTROL",
            font=("Arial", 12, "bold"),
            bg="#34495e",
            fg="white",
            padx=10,
            pady=10
        )
        control_frame.pack(fill="x", padx=10, pady=10)
        
        # D-pad style control
        btn_grid = tk.Frame(control_frame, bg="#34495e")
        btn_grid.pack(pady=10)
        
        btn_style = {"font": ("Arial", 10, "bold"), "width": 8, "height": 2, "relief": "raised"}
        
        tk.Button(btn_grid, text="⬆", command=lambda: self.manual_control(0, 50), 
                 bg="#16a085", fg="white", **btn_style).grid(row=0, column=1, padx=2, pady=2)
        tk.Button(btn_grid, text="⬅", command=lambda: self.manual_control(-50, 50),
                 bg="#16a085", fg="white", **btn_style).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(btn_grid, text="STOP", command=lambda: self.manual_control(0, 0),
                 bg="#e74c3c", fg="white", **btn_style).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(btn_grid, text="➡", command=lambda: self.manual_control(50, 50),
                 bg="#16a085", fg="white", **btn_style).grid(row=1, column=2, padx=2, pady=2)
        tk.Button(btn_grid, text="⬇", command=lambda: self.manual_control(0, -50),
                 bg="#16a085", fg="white", **btn_style).grid(row=2, column=1, padx=2, pady=2)
        
        # Exit button (bottom)
        tk.Button(
            parent,
            text="❌ EXIT",
            command=self.root.quit,
            font=("Arial", 10),
            bg="#c0392b",
            fg="white",
            relief="flat",
            pady=5
        ).pack(side="bottom", fill="x", padx=10, pady=10)
    
    def build_map(self, parent):
        """Build map widget"""
        self.map_widget = tkintermapview.TkinterMapView(parent, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        
        # Load default location from config
        default_lat = self.config.get('map', 'default_lat', default=35.7796)
        default_lon = self.config.get('map', 'default_lon', default=-78.6382)
        default_zoom = self.config.get('map', 'default_zoom', default=15)
        
        self.map_widget.set_position(default_lat, default_lon)
        self.map_widget.set_zoom(default_zoom)
        
        # Boat marker
        self.boat_marker = None
        self.waypoint_marker = None
        self.track_path = None
        
        # Click to set waypoint
        self.map_widget.add_left_click_map_command(self.map_clicked)
    
    def map_clicked(self, coords):
        """Handle map click to set waypoint"""
        lat, lon = coords
        
        # Set waypoint marker
        if self.waypoint_marker:
            self.waypoint_marker.delete()
        
        waypoint_color = self.config.get('map', 'waypoint_marker_color', default='red')
        
        self.waypoint_marker = self.map_widget.set_marker(
            lat, lon,
            text="🎯 Target",
            marker_color_circle=waypoint_color,
            marker_color_outside="darkred"
        )
        
        # Send waypoint to boat
        self.send_waypoint(lat, lon)
    
    # === COMMUNICATION METHODS ===
    
    def toggle_connection(self):
        """Connect/disconnect serial"""
        if not self.is_connected:
            try:
                self.ser = serial.Serial(
                    self.serial_port,
                    self.baud_rate,
                    timeout=self.serial_timeout
                )
                self.is_connected = True
                
                self.btn_connect.config(text="⚡ DISCONNECT", bg="#e74c3c")
                self.lbl_status.config(text="CONNECTED", fg="#27ae60")
                
                # Start RX thread
                threading.Thread(target=self.serial_rx_thread, daemon=True).start()
                
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to connect:\n{e}")
        else:
            self.is_connected = False
            if self.ser:
                self.ser.close()
            
            self.btn_connect.config(text="⚡ CONNECT", bg="#27ae60")
            self.lbl_status.config(text="DISCONNECTED", fg="#e74c3c")
    
    def serial_rx_thread(self):
        """Background thread for receiving data"""
        buffer = b''
        
        while self.is_connected:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer += data
                    
                    # Try to parse telemetry packet (22 bytes)
                    if len(buffer) >= 22:
                        telemetry = self.protocol.unpack_telemetry(buffer[:22])
                        if telemetry:
                            # Check for RSSI metadata
                            if b'RSSI:' in buffer:
                                try:
                                    rssi_str = buffer.split(b'RSSI:')[1].split(b'\n')[0]
                                    telemetry.rssi = int(rssi_str)
                                except:
                                    pass
                            
                            self.telemetry_queue.put(telemetry)
                            buffer = buffer[22:]
                        else:
                            buffer = buffer[1:]
                    
                    # Prevent buffer overflow
                    if len(buffer) > 1000:
                        buffer = buffer[-100:]
                
            except Exception as e:
                print(f"RX Error: {e}")
                break
            
            time.sleep(0.01)
    
    def send_waypoint(self, lat: float, lon: float):
        """Send waypoint to boat"""
        if self.is_connected and self.ser:
            packet = self.protocol.pack_waypoint(lat, lon)
            self.ser.write(packet)
            print(f"Sent waypoint: {lat:.6f}, {lon:.6f}")
    
    def enable_autonomous_mode(self):
        """Switch boat to autonomous mode"""
        if self.is_connected and self.ser:
            packet = self.protocol.pack_mode(MODE_AUTONOMOUS)
            self.ser.write(packet)
            self.current_mode = MODE_AUTONOMOUS
            self.lbl_mode.config(text="MODE: AUTONOMOUS", fg="#27ae60")
            print("Switched to AUTONOMOUS mode")
    
    def enable_manual_mode(self):
        """Switch boat to manual mode"""
        if self.is_connected and self.ser:
            packet = self.protocol.pack_mode(MODE_MANUAL)
            self.ser.write(packet)
            self.current_mode = MODE_MANUAL
            self.lbl_mode.config(text="MODE: MANUAL", fg="#f39c12")
            print("Switched to MANUAL mode")
    
    def manual_control(self, rudder: int, sail: int):
        """Send manual control command"""
        if self.is_connected and self.ser:
            packet = self.protocol.pack_control(rudder, sail)
            self.ser.write(packet)
            print(f"Manual control: Rudder={rudder}, Sail={sail}")
    
    # === UI UPDATE ===
    
    def update_ui(self):
        """Periodic UI update from telemetry queue"""
        
        # Process telemetry queue
        try:
            while not self.telemetry_queue.empty():
                telemetry = self.telemetry_queue.get_nowait()
                self.current_telemetry = telemetry
                
                # Update labels
                self.lbl_position.config(text=f"GPS: {telemetry.lat:.6f}°, {telemetry.lon:.6f}°")
                self.lbl_heading.config(text=f"HDG: {telemetry.heading:.1f}°")
                self.lbl_wind.config(text=f"WIND: {telemetry.wind_angle:.1f}° (rel)")
                
                # Battery with color coding
                bat_text = f"BAT: {telemetry.battery:.2f} V"
                if telemetry.battery < self.battery_crit:
                    self.lbl_battery.config(text=bat_text, fg="#e74c3c")  # Red
                elif telemetry.battery < self.battery_warn:
                    self.lbl_battery.config(text=bat_text, fg="#f39c12")  # Orange
                else:
                    self.lbl_battery.config(text=bat_text, fg="#27ae60")  # Green
                
                # RSSI
                self.lbl_rssi.config(text=f"RSSI: {telemetry.rssi} dBm")
                
                # Update map
                if telemetry.lat != 0 and telemetry.lon != 0:
                    if self.boat_marker:
                        self.boat_marker.set_position(telemetry.lat, telemetry.lon)
                    else:
                        self.boat_marker = self.map_widget.set_marker(
                            telemetry.lat, telemetry.lon,
                            text="⛵ Boat",
                            marker_color_circle="blue",
                            marker_color_outside="darkblue"
                        )
                    
                    # Center map on boat (if configured)
                    if self.config.get('map', 'auto_center_on_boat', default=True):
                        self.map_widget.set_position(telemetry.lat, telemetry.lon)
                    
                    # Add to track
                    self.track_points.append((telemetry.lat, telemetry.lon, time.time()))
                    self.update_track()
        
        except queue.Empty:
            pass
        
        # Schedule next update
        self.root.after(self.update_rate_ms, self.update_ui)
    
    def update_track(self):
        """Draw track path on map"""
        if len(self.track_points) < 2:
            return
        
        # Keep max points from config
        if len(self.track_points) > self.track_max_points:
            self.track_points = self.track_points[-self.track_max_points:]
        
        # Draw path with config colors
        if self.track_path:
            self.track_path.delete()
        
        coords = [(p[0], p[1]) for p in self.track_points]
        
        self.track_path = self.map_widget.set_path(
            coords, 
            color=self.track_color, 
            width=self.track_width
        )
    
    # === SETTINGS DIALOG ===
    
    def show_settings_dialog(self):
        """Open settings configuration dialog"""
        settings_win = tk.Toplevel(self.root)
        settings_win.title("⚙️ Settings")
        settings_win.geometry("500x600")
        settings_win.configure(bg="#34495e")
        
        # Serial settings
        serial_frame = tk.LabelFrame(
            settings_win, 
            text="Serial Port", 
            bg="#34495e", 
            fg="white",
            font=("Arial", 11, "bold"),
            padx=10, 
            pady=10
        )
        serial_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Label(serial_frame, text="Port:", bg="#34495e", fg="white").grid(row=0, column=0, sticky="w", pady=5)
        port_entry = tk.Entry(serial_frame, width=30)
        port_entry.insert(0, self.config.get('serial', 'port'))
        port_entry.grid(row=0, column=1, pady=5)
        
        tk.Label(serial_frame, text="Baud Rate:", bg="#34495e", fg="white").grid(row=1, column=0, sticky="w", pady=5)
        baud_entry = tk.Entry(serial_frame, width=30)
        baud_entry.insert(0, str(self.config.get('serial', 'baud_rate')))
        baud_entry.grid(row=1, column=1, pady=5)
        
        # Map settings
        map_frame = tk.LabelFrame(
            settings_win, 
            text="Map", 
            bg="#34495e", 
            fg="white",
            font=("Arial", 11, "bold"),
            padx=10, 
            pady=10
        )
        map_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Label(map_frame, text="Track Color:", bg="#34495e", fg="white").grid(row=0, column=0, sticky="w", pady=5)
        color_var = tk.StringVar(value=self.config.get('map', 'track_color'))
        color_menu = ttk.Combobox(
            map_frame, 
            textvariable=color_var, 
            values=["blue", "red", "green", "orange", "purple", "black"],
            width=27
        )
        color_menu.grid(row=0, column=1, pady=5)
        
        tk.Label(map_frame, text="Max Track Points:", bg="#34495e", fg="white").grid(row=1, column=0, sticky="w", pady=5)
        track_entry = tk.Entry(map_frame, width=30)
        track_entry.insert(0, str(self.config.get('map', 'track_max_points')))
        track_entry.grid(row=1, column=1, pady=5)
        
        tk.Label(map_frame, text="Track Width:", bg="#34495e", fg="white").grid(row=2, column=0, sticky="w", pady=5)
        width_entry = tk.Entry(map_frame, width=30)
        width_entry.insert(0, str(self.config.get('map', 'track_width')))
        width_entry.grid(row=2, column=1, pady=5)
        
        # Telemetry settings
        telem_frame = tk.LabelFrame(
            settings_win, 
            text="Telemetry Alerts", 
            bg="#34495e", 
            fg="white",
            font=("Arial", 11, "bold"),
            padx=10, 
            pady=10
        )
        telem_frame.pack(fill="x", padx=10, pady=5)
        
        tk.Label(telem_frame, text="Battery Warning (V):", bg="#34495e", fg="white").grid(row=0, column=0, sticky="w", pady=5)
        bat_warn_entry = tk.Entry(telem_frame, width=30)
        bat_warn_entry.insert(0, str(self.config.get('telemetry', 'battery_warn_voltage')))
        bat_warn_entry.grid(row=0, column=1, pady=5)
        
        tk.Label(telem_frame, text="Battery Critical (V):", bg="#34495e", fg="white").grid(row=1, column=0, sticky="w", pady=5)
        bat_crit_entry = tk.Entry(telem_frame, width=30)
        bat_crit_entry.insert(0, str(self.config.get('telemetry', 'battery_critical_voltage')))
        bat_crit_entry.grid(row=1, column=1, pady=5)
        
        # Button frame
        btn_frame = tk.Frame(settings_win, bg="#34495e")
        btn_frame.pack(pady=20)
        
        # Save button
        def save_settings():
            try:
                self.config.set('serial', 'port', value=port_entry.get())
                self.config.set('serial', 'baud_rate', value=int(baud_entry.get()))
                self.config.set('map', 'track_color', value=color_var.get())
                self.config.set('map', 'track_max_points', value=int(track_entry.get()))
                self.config.set('map', 'track_width', value=int(width_entry.get()))
                self.config.set('telemetry', 'battery_warn_voltage', value=float(bat_warn_entry.get()))
                self.config.set('telemetry', 'battery_critical_voltage', value=float(bat_crit_entry.get()))
                
                self.config.save_config()
                
                messagebox.showinfo("Settings", "Settings saved!\n\nRestart the app to apply serial port changes.")
                settings_win.destroy()
            except ValueError as e:
                messagebox.showerror("Invalid Input", f"Please check your values:\n{e}")
        
        # Cancel button
        def cancel_settings():
            settings_win.destroy()
        
        tk.Button(
            btn_frame, 
            text="💾 Save", 
            command=save_settings, 
            bg="#27ae60", 
            fg="white",
            font=("Arial", 11, "bold"),
            padx=20, 
            pady=8,
            relief="flat"
        ).pack(side="left", padx=5)
        
        tk.Button(
            btn_frame, 
            text="❌ Cancel", 
            command=cancel_settings, 
            bg="#95a5a6", 
            fg="white",
            font=("Arial", 11, "bold"),
            padx=20, 
            pady=8,
            relief="flat"
        ).pack(side="left", padx=5)

# === MAIN ENTRY POINT ===

if __name__ == "__main__":
    root = tk.Tk()
    app = MissionControlApp(root)
    root.mainloop()
