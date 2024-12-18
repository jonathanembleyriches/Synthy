
import unreal
# Load the Blueprint asset
bp_asset = unreal.load_asset('/Game/NewBlueprint.NewBlueprint')
# Function to get the currently selected asset in the Content Browser
def get_selected_assets_in_content_browser():
    selected_assets = unreal.EditorUtilityLibrary.get_selected_assets()
    if not selected_assets:
        print("No asset selected in the Content Browser!")
        return None

    for asset in selected_assets:
        print(f"Selected Asset: {asset.get_path_name()}")
        return asset  # Return the first selected asset for simplicity
selected_bp =   get_selected_assets_in_content_browser()
print(selected_bp)
if selected_bp:

    print(f"You selected: {selected_bp}")
    bp_asset = selected_bp
# Get the SubobjectDataSubsystem and gather subobject data for the Blueprint
subsystem: unreal.SubobjectDataSubsystem = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
root_data_handle: unreal.SubobjectDataHandle = subsystem.k2_gather_subobject_data_for_blueprint(context=bp_asset)

objects = []
# Gather all components in the Blueprint
for handle in root_data_handle:
    subobject = subsystem.k2_find_subobject_data_from_handle(handle)
    objects.append(unreal.SubobjectDataBlueprintFunctionLibrary.get_object(subobject))

# Remove duplicate references
objects = list(dict.fromkeys(objects))

# Iterate over objects and check if they are StaticMeshComponents
for obj in objects:
    if "StaticMeshComponent" in str(obj):
        static_mesh = obj.get_editor_property("static_mesh")
        print(f"StaticMeshComponent: {obj}, StaticMesh: {static_mesh}")

        if str(static_mesh) == "None":
            print(f"StaticMeshComponent: {obj} has no static_mesh")
        else:
            # Clean up the component name
            component_name = obj.get_name()
            clean_name = component_name.replace("_GEN_VARIABLE", "")
            
            # Retrieve existing tags
            existing_tags = obj.get_editor_property("component_tags")

            if clean_name not in existing_tags:
                updated_tags =  [clean_name]
                obj.set_editor_property("component_tags", updated_tags)

                print(f"Added tag '{clean_name}' to StaticMeshComponent: {obj}")
            else:
                print(f"Tag '{clean_name}' already exists for StaticMeshComponent: {obj}")
