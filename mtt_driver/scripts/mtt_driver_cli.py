#!/usr/bin/env python3
"""Command-line interface for MTT CAN driver with system logging control."""

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))
from mtt_driver.mtt_driver import MTTCanDriver


def signal_handler(signum, frame):
    """Handle SIGINT gracefully."""
    print("\nShutting down MTT driver...")
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='MTT CAN Driver CLI')
    
    parser.add_argument(
        '--can-interface', 
        default='can0',
        help='CAN interface name (default: can0)'
    )
    
    parser.add_argument(
        '--log-level',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        default='INFO',
        help='Logging level (default: INFO)'
    )
    
    parser.add_argument(
        '--test-mode',
        action='store_true',
        help='Run basic driver test and exit'
    )
    
    args = parser.parse_args()
    
    log_level = getattr(logging, args.log_level.upper())
    
    print(f"Starting MTT CAN Driver on {args.can_interface} with log level {args.log_level}")
    
    try:
        driver = MTTCanDriver(can_interface=args.can_interface, log_level=log_level)
        
        if args.test_mode:
            print("Running basic test...")
            print(f"E-STOP status: {driver.estop_active}")
            driver.release_estop()
            print(f"E-STOP released: {not driver.estop_active}")
            
            driver.set_throttle_percent(0.1)
            driver.set_steer_normalized(0.0)
            driver.set_brake_percent(0.0)
            
            print("Basic test completed successfully")
            driver.cleanup()
            return
        
        signal.signal(signal.SIGINT, signal_handler)
        
        print("Driver running... Press Ctrl+C to stop")
        print("Use --test-mode for basic functionality test")
        
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
            
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        try:
            driver.cleanup()
        except:
            pass


if __name__ == '__main__':
    main()
