from pydrake.all import ModelVisualizer

from wimp.scene import start_meshcat

def main():
    # 1. Start the Meshcat server in the background (start_meshcat also works
    #    inside Docker, where Meshcat must listen on all interfaces)
    meshcat = start_meshcat()

    # 2. Initialize the visualizer tool
    visualizer = ModelVisualizer(meshcat=meshcat)

    # 3. Register your local 'models' folder as a Drake package
    # This matches the 'package://custom_models/...' URIs in your YAML
    visualizer.parser().package_map().Add("custom_models", "./models")

    # 4. Parse the YAML assembly directive
    visualizer.AddModels("models/mobile_manipulator.dmd.yaml")

    print("\n" + "="*50)
    print("Model loaded successfully!")
    print("Open the Meshcat URL printed above in your web browser.")
    print("Expand the 'Animations' control panel in Meshcat to use the joint sliders.")
    print("="*50 + "\n")

    # 5. Start the interactive loop (blocks the terminal until you press Ctrl+C)
    visualizer.Run(loop_once=False)

if __name__ == "__main__":
    main()
