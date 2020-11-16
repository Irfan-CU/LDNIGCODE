# LDMIGCODE
FDM Toolpath Code Overview:

The most Important Part in this code for generating LDNI FDM Toolpaths is.   
1-Call to Slices Functions  in PMBody.Cpp. Real code for call to slice model is below in line.   

"bool slice_model= polygongenrator.sliceModel(VSAMeshList, c_mesh, storage,iRes[1], meshin_layer, rotBoundingBox);"   

2-SliceModel function add contours in a slicer list and form parts form the input polygons.    

3-After SliceModel Slices2Polygons is called in PMBody.cpp   

4- Slices2Polygons is the main function which utilized the polygon outlines to develop   
     1-processBasicWallsSkinInfill (storage, mesh_order_idx, mesh_order, inset_skin_progress_estimate);   
     2-This Basic Infill Process generate Insets form the outline and generate Skin and Infill areas inside the insets.    
     3-After Basic Infill platform adhesion function process SkirtBrim.     

5-After this the most Important Function works which is write GCode    
      1-Write GCode first process all layers and develops the infill for all the layers.   
      2-After this each layer is processed separately to write GCode to the final output file.    

# Errors:
1-The coordinates are offset as currently the code is working on relative coordinates w.r.t B.Box of the part. This need to be changed to abs coordinates of the 3D Printer.

# Solved:
1- Solved the acceleration issues.   
2- Solved the extrusion values which were scaled by 10000 by introducing the Ratio class which reduced the line width factory by 100 and now the extrusion values are just like any other Gcode Path obtained.   
3- Solved the negative (-) Extrusion values related to Gcode Path.   
4- Solved the Repeating reset command of extrusion which was also related to the scaled vales of the extrusion.   
5- Working on changing the coordinates and reading research for FGM in LDNI.    
6- Amf reading is done in LDNI.
7- AMG material reading is done in LDNI.
8- So Normal values are changed in RGB of the texture and so getting the desired values on the sample points.
9- LDNI of AMF file is successfully done so any input AMF file can be processes for LDNI sampling now.
10-Input of the Dual Mesh Input working perfectly now.
11-Dual Mesh AMF LDNI is working as planned.

# Update
1-

# Current Work
1- Working on developing of the algorithm to read the material info from the AMF and then to texture and then samples on ray.

      
      
     

