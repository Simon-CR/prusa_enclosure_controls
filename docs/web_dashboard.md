# Web Dashboard & Integration Guide

## Accessing the Dashboard

1. Connect controller to WiFi (configure in firmware)
2. Navigate to: `http://prusa-enclosure.local`
3. Dashboard updates in real-time

## Home Assistant Setup

1. Configure MQTT broker
2. Update firmware with broker details
3. System auto-discovers to HA

## REST API Reference

### GET /api/status
Returns current system state

### POST /api/command
Send control commands

### GET /api/history
Returns historical data

## MQTT Topics
- `prusa/enclosure/state` - Status updates
- `prusa/enclosure/command` - Commands
- `prusa/enclosure/availability` - Online status
