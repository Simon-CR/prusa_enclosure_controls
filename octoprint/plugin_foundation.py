#!/usr/bin/env python3
"""
OctoPrint Plugin Foundation for Prusa Enclosure Controller
"""

import requests
import json

class PrusaEnclosureAPI:
    """Foundation class for OctoPrint plugin developers"""
    
    def __init__(self, controller_ip="prusa-enclosure.local"):
        self.base_url = f"http://{controller_ip}"
        self.api_endpoint = f"{self.base_url}/api"
        
    def get_status(self):
        """Get current enclosure status"""
        try:
            response = requests.get(f"{self.api_endpoint}/status", timeout=5)
            return response.json()
        except Exception as e:
            return {"error": str(e)}
    
    def send_command(self, command, **kwargs):
        """Send command to enclosure"""
        payload = {"command": command}
        payload.update(kwargs)
        
        try:
            response = requests.post(
                f"{self.api_endpoint}/command",
                json=payload,
                timeout=5
            )
            return response.json()
        except Exception as e:
            return {"error": str(e)}
    
    def set_profile_for_material(self, material_name):
        """Set profile based on material"""
        profiles = {
            "PLA": 1, "PETG": 2, "TPU": 3,
            "NYLON": 4, "ABS": 5, "ASA": 6, "PC": 7
        }
        profile_id = profiles.get(material_name.upper(), 0)
        return self.send_command("start", profile=profile_id)

# Example usage in OctoPrint plugin:
# api = PrusaEnclosureAPI()
# status = api.get_status()
# api.set_profile_for_material("ABS")
