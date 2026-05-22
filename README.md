# Code base for Hexapod, v3.

6 legs, 18DOF hexapod robot.
18x HiWonder HPS2018/2027 20kg*cm servo motors controlled by Arduino Mega and UNO-based RC controller.

Implementing as many gaits as I can.

# Repository Contents:
## Design
- Design Overview
- Full BOM (CSV)
- Mechanical Design 
    - Mechanical BOM (CSV)
    - Photos
        - CAD Photos
        - Build Photos
            OnShape
                Link, Design Choices/Features (loft, sweep, rib, extend) 
                Variable Tables (exported) 
            Kinematics 
                FK Diagram and Equations
                IK Diagram and Equations
        Electrical Design
            Electrical BOM (CSV)
            Schematic Layout
            PCB Layout
            Photos
                EasyEDA (link, design choices)
                Build Photos
            Electrical Specifications (single leg/total current draw, voltage stability study with caps, radio frequencies)
        Software Design
            UML Diagram
            Design choices
    Code
        src
        lib
        platformio.ini
        push.sh
        ...
            
    