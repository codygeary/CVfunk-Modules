#include "plugin.hpp"


Plugin* pluginInstance;


void init(Plugin* p) {
    pluginInstance = p;

    // Add modules here
    p->addModel(modelSteps);
    p->addModel(modelEnvelopeArray);
    p->addModel(modelPentaSequencer);
    p->addModel(modelImpulseController);
    p->addModel(modelSignals);
    p->addModel(modelRanges);
    p->addModel(modelHexMod);
    p->addModel(modelCollatz);
    p->addModel(modelStrings);
    p->addModel(modelMagnets);
    p->addModel(modelOuros);
    p->addModel(modelPressedDuck);
    p->addModel(modelFlowerPatch);
    p->addModel(modelSyncro);
    p->addModel(modelNona);
    p->addModel(modelDecima);
    p->addModel(modelMorta);
    p->addModel(modelStepWave);
    p->addModel(modelPreeeeeeeeeeessedDuck);
    p->addModel(modelArrange);
    p->addModel(modelTriDelay);
    p->addModel(modelTatami);
    p->addModel(modelCartesia);
    p->addModel(modelJunkDNA);
    p->addModel(modelPicus);
    p->addModel(modelNode);
    p->addModel(modelWeave);
    p->addModel(modelWonk);
    p->addModel(modelHammer);
    p->addModel(modelHub);
    p->addModel(modelCVfunkBlank);
    p->addModel(modelCVfunkBlank4HP);
    p->addModel(modelRat);
    p->addModel(modelCount);

    
    // Any other plugin initialization may go here.
    // As an alternative, consider lazy-loading assets and lookup tables when your module is created to reduce startup times of Rack.
}
