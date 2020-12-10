# LDNIGCODE
FDM Toolpath Code OverView:

The most Important Part in this code for genrating LDMI FDM Toolpaths is.   

This is a parallel slicing tool for advaced multimaterial AM applications. It generates a toolpath for FDM addtive manufacturing for overlapping materials and to improve the strength of multimaterial parts at the interfacial boundary by develloping mixed infill and interlocking joints automatically withou any modification in CAD design.

# Solved Issues:

1- Solved the accelration issues.   
2- Solved the extrusion values which were scaled by 10000 by introducing the Ratio class which reduced the line width factory by 100 and now the extrusion values are just like any other Gcode Path obtained.   
3- Solved the negative (-) Extrusion values related to Gcode Path.   
4- Solved the Repeating reset command of extrusion which was also related to the scaled vales of the exxtrusion.     

# Progress:

Developed Parallel LDMI (Layer Depth Material Imaging Slicer) for AM.

# Current Tasks

Improvement ad testing of the developed tool.

      
      
     

