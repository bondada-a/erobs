#!/usr/bin/env python3
"""
Automatically run Zivid Capture Assistant and save optimized settings
"""
import rclpy
from rclpy.node import Node
from zivid_interfaces.srv import CaptureAssistantSuggestSettings
from builtin_interfaces.msg import Duration
import yaml
import sys
import os

class ZividAutoConfig(Node):
    def __init__(self):
        super().__init__('zivid_auto_config')
        self.client = self.create_client(
            CaptureAssistantSuggestSettings,
            '/capture_assistant/suggest_settings'
        )

    def run_capture_assistant(self, max_time_sec=2, lighting_freq=2):
        """
        Run Capture Assistant
        lighting_freq: 0=None, 1=50Hz (Europe), 2=60Hz (US)
        """
        self.get_logger().info('Waiting for Capture Assistant service...')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Capture Assistant service not available')
            return None

        # Create request
        request = CaptureAssistantSuggestSettings.Request()
        request.max_capture_time = Duration(sec=max_time_sec, nanosec=0)
        request.ambient_light_frequency = lighting_freq

        self.get_logger().info(f'Running Capture Assistant (max time: {max_time_sec}s)...')
        self.get_logger().info('This will take 5-10 seconds - analyzing your scene...')

        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is None:
            self.get_logger().error('Capture Assistant call failed')
            return None

        result = future.result()
        if not result.success:
            self.get_logger().error(f'Capture Assistant failed: {result.message}')
            return None

        self.get_logger().info('✓ Capture Assistant completed successfully')
        return result.suggested_settings

    def extract_2d_settings(self, full_settings):
        """Extract Settings2D from full settings YAML"""
        try:
            settings = yaml.safe_load(full_settings)

            if 'Settings' not in settings:
                self.get_logger().error('No Settings found in response')
                return None

            if 'Color' not in settings['Settings']:
                self.get_logger().error('No Color settings found')
                return None

            if 'Settings2D' not in settings['Settings']['Color']:
                self.get_logger().error('No Settings2D found')
                return None

            # Extract Settings2D section
            settings_2d = {
                '__version__': settings['Settings']['Color']['__version__'],
                'Settings2D': settings['Settings']['Color']['Settings2D']
            }

            # For 2D capture, use only first acquisition (avoid HDR aperture issues)
            if len(settings_2d['Settings2D']['Acquisitions']) > 1:
                self.get_logger().warn('Multiple acquisitions detected - using only first (2D HDR aperture limitation)')
                settings_2d['Settings2D']['Acquisitions'] = [
                    settings_2d['Settings2D']['Acquisitions'][0]
                ]

            return settings_2d

        except Exception as e:
            self.get_logger().error(f'Failed to parse settings: {e}')
            return None

    def save_settings(self, settings_2d, output_file):
        """Save Settings2D to YAML file"""
        try:
            with open(output_file, 'w') as f:
                yaml.dump(settings_2d, f, default_flow_style=False, sort_keys=False)

            self.get_logger().info(f'✓ Settings saved to: {output_file}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save settings: {e}')
            return False

    def update_camera_param(self, settings_file):
        """Update camera parameter to use new settings"""
        self.get_logger().info('Updating camera to use new settings...')

        # Create parameter client
        param_client = self.create_client(
            rcl_interfaces.srv.SetParameters,
            '/zivid_camera/set_parameters'
        )

        # For now, just print the command
        self.get_logger().info(f'\nTo apply settings, run:')
        self.get_logger().info(f'  ros2 param set /zivid_camera settings_2d_file_path {settings_file}')

def main(args=None):
    rclpy.init(args=args)

    # Parse arguments
    lighting = 2  # Default 60Hz (US)
    if len(sys.argv) > 1:
        if sys.argv[1] == '50hz':
            lighting = 1
        elif sys.argv[1] == 'none':
            lighting = 0

    # Determine output file path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = os.path.join(script_dir, 'src/zivid-ros/cam_settings_2d_auto.yml')

    node = ZividAutoConfig()

    print("\n" + "="*60)
    print("  Zivid Automatic Configuration")
    print("="*60)

    # Step 1: Run Capture Assistant
    full_settings = node.run_capture_assistant(max_time_sec=2, lighting_freq=lighting)
    if full_settings is None:
        print("\n✗ FAILED: Could not get settings from Capture Assistant")
        return 1

    # Step 2: Extract 2D settings
    settings_2d = node.extract_2d_settings(full_settings)
    if settings_2d is None:
        print("\n✗ FAILED: Could not extract 2D settings")
        return 1

    # Step 3: Save to file
    if not node.save_settings(settings_2d, output_file):
        print("\n✗ FAILED: Could not save settings file")
        return 1

    # Step 4: Show summary
    acquisitions = settings_2d['Settings2D']['Acquisitions'][0]['Acquisition']
    print("\n" + "="*60)
    print("  ✓ SUCCESS - Optimized Settings Saved")
    print("="*60)
    print(f"  File: {output_file}")
    print(f"\n  Optimized Parameters:")
    print(f"    Aperture:     f/{acquisitions['Aperture']}")
    print(f"    ExposureTime: {acquisitions['ExposureTime']}μs")
    print(f"    Gain:         {acquisitions['Gain']:.2f}")
    print(f"    Brightness:   {acquisitions['Brightness']}")
    print("\n  To apply settings:")
    print(f"    ros2 param set /zivid_camera settings_2d_file_path {output_file}")
    print("="*60 + "\n")

    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())
