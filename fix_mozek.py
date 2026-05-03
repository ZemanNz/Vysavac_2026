with open('ESP32-detekce/src/mozek.h', 'r') as f:
    content = f.read()

start_marker = "// =============================================================================\n//  POMOCNÉ FUNKCE PRO ROTACI A KOREKCI (LiDAR)"
end_marker = "// =============================================================================\n//  UART FUNKCE"

start_idx = content.find(start_marker)
end_idx = content.find(end_marker)

if start_idx != -1 and end_idx != -1:
    # Extract the block
    helpers_block = content[start_idx:end_idx]
    
    # Remove the block from its current location
    content = content[:start_idx] + content[end_idx:]
    
    # Find the injection point (before mozek_rozhoduj)
    inject_marker = "// Forward deklarace (definice je níže)\nvoid mozek_start_zapasu();"
    inject_idx = content.find(inject_marker)
    
    if inject_idx != -1:
        # Insert the block
        content = content[:inject_idx] + helpers_block + "\n" + content[inject_idx:]
        
        with open('ESP32-detekce/src/mozek.h', 'w') as f:
            f.write(content)
        print("Fix applied successfully.")
    else:
        print("Could not find inject marker.")
else:
    print("Could not find helpers block.")
