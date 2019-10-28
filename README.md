# LDNIGCODE
FDM Toolpath Code OverView:

The most Important Part in this code for genrating LDNI FDM Toolpaths is.   
1-Call to Slices Functions  in PMBody.Cpp. Real code for call to slice model is below in line.   

"bool slice_model= polygongenrator.sliceModel(VSAMeshList, c_mesh, storage,iRes[1], meshin_layer, rotBoundingBox);"   

2-SliceModel function add contours in a slicer list and form parts form the input polygons.    

3-After SliceModel Slices2Polygons is called in PMBody.cpp   

4- Slices2Polygons is the main function which utlized the polygon outlines to develop   
     1-processBasicWallsSkinInfill(storage, mesh_order_idx, mesh_order, inset_skin_progress_estimate);   
     2-This Basic Infill Process generateInsets form the outline and genrate Skin and Infill areas insid the insets.    
     3-After Basic Infill platform adhesion function process SkirtBrim.     

5-After this the most Important Function works which is writeGCode    
      1-Write GCode first process all layers and develops the infill for all the layers.   
      2-After this each layer is processed seprately to write GCode to the final output file.    
     

# Errors:

1-The coordinates are offset as righnow the code is working on relative coordinates wrt BBox of the part. This need to be changed to abs coordinates of the 3D Printer.

# Solved Issues:

1- Solved the accelration issues.   
2- Solved the extrusion values which were scaled by 10000 by introducing the Ratio class which reduced the line width factory by 100 and now the extrusion values are just like any other Gcode Path obtained.   
3- Solved the negative (-) Extrusion values related to Gcode Path.   
4- Solved the Repeating reset command of extrusion which was also related to the scaled vales of the exxtrusion.     

# Progress:

1- Solved the issues (2-4) mentioned in above lines.   

# Current Work

1- Working on changing the coordinates and reading research for FGM in LDNI.    

      
      
     

