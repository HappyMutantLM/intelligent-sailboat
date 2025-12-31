"""
Intelligent Sailboat - Base Station GUI
Raspberry Pi 4 with 7" Touchscreen
Communicates with boat via Feather M0 LoRa USB dongle
"""

import tkinter as tk
from tkinter import ttk, messagebox
import tkintermapview
import serial
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional
import queue

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyACM0'  # Feather M0 USB
BAUD_RATE = 115200

# === PROTOCOL DEFINITIONS (Match Protocol.h) ===
PKT_CONTROL = 0x01
PKT_TELEMETRY = 0x02
PKT_WAYPOINT = 0x03
PKT_MODE_CHANGE = 0x04

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
        packet = struct.pack('<Bfff', PKT_WAYPOINT, lat, lon, 0)  # checksum=0 for now
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
        if len(data) < 22:  # TelemetryPacket size
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

class MissionControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Intelligent Sailboat - Base Station")
        
        # Fullscreen for 7" touchscreen (720x1280)
        self.root.attributes('-fullscreen', True)
        # Or windowed for development:
        # self.root.geometry("1280x720")
        
        self.ser: Optional[serial.Serial] = None
        self.is_connected = False
        self.telemetry_queue = queue.Queue()
        self.track_points = []  # Store (lat, lon, timestamp) for track history
        
        # Protocol handler
        self.protocol = ProtocolHandler()
        
        # Current boat state
        self.current_telemetry = TelemetryData()
        
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
        
        # Default location (will update when GPS fix)
        self.map_widget.set_position(35.7796, -78.6382)  # Raleigh, NC
        self.map_widget.set_zoom(15)
        
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
        
        self.waypoint_marker = self.map_widget.set_marker(
            lat, lon,
            text="Target",
            marker_color_circle="red",
            marker_color_outside="darkred"
        )
        
        # Send waypoint to boat
        self.send_waypoint(lat, lon)
    
    # === COMMUNICATION METHODS ===
    
    def toggle_connection(self):
        """Connect/disconnect serial"""
        if not self.is_connected:
            try:
                self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
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
                                rssi_str = buffer.split(b'RSSI:')[1].split(b'\n')[0]
                                telemetry.rssi = int(rssi_str)
                            
                            self.telemetry_queue.put(telemetry)
                            buffer = buffer[22:]  # Remove parsed packet
                        else:
                            buffer = buffer[1:]  # Shift by 1 byte and try again
                    
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
            self.lbl_mode.config(text="MODE: AUTONOMOUS", fg="#27ae60")
            print("Switched to AUTONOMOUS mode")
    
    def enable_manual_mode(self):
        """Switch boat to manual mode"""
        if self.is_connected and self.ser:
            packet = self.protocol.pack_mode(MODE_MANUAL)
            self.ser.write(packet)
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
                self.lbl_battery.config(text=f"BAT: {telemetry.battery:.2f} V")
                self.lbl_rssi.config(text=f"RSSI: {telemetry.rssi} dBm")
                
                # Update map
                if telemetry.lat != 0 and telemetry.lon != 0:
                    if self.boat_marker:
                        self.boat_marker.set_position(telemetry.lat, telemetry.lon)
                    else:
                        self.boat_marker = self.map_widget.set_marker(
                            telemetry.lat, telemetry.lon,
                            text="⛵ Boat",
                            icon=self.create_boat_icon(telemetry.heading)
                        )
                    
                    # Center map on boat
                    self.map_widget.set_position(telemetry.lat, telemetry.lon)
                    
                    # Add to track
                    self.track_points.append((telemetry.lat, telemetry.lon, time.time()))
                    self.update_track()
        
        except queue.Empty:
            pass
        
        # Schedule next update
        self.root.after(100, self.update_ui)  # 10 Hz
    
    def update_track(self):
        """Draw track path on map"""
        if len(self.track_points) < 2:
            return
        
        # Keep last 500 points (configurable)
        if len(self.track_points) > 500:
            self.track_points = self.track_points[-500:]
        
        # Draw path
        if self.track_path:
            self.track_path.delete()
        
        coords = [(p[0], p[1]) for p in self.track_points]
        self.track_path = self.map_widget.set_path(coords, color="blue", width=3)
    
    def create_boat_icon(self, heading: float):
        """Create rotated boat icon (optional - needs PIL)"""
        # For now, just use default marker
        # TODO: Implement rotated icon with PIL
        return None

if __name__ == "__main__":
    root = tk.Tk()
    app = MissionControlApp(root)
    root.mainloop()
