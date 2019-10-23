#pragma once


#ifndef LAYERPART_H
#define LAYERPART_H


class SliceLayer;
class Slicer;
class SlicerLayer;
class SliceDataStorage;

void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer);
void createLayerParts(SliceDataStorage& storage, Slicer* slicer);



	
	

#endif
