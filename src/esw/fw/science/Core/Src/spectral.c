#include "spectral.h"
// REQUIRES: SMBus declared by user
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(SMBus *smbus){
    Spectral* spectral = malloc(sizeof(Spectral));
    spectral->smbus = smbus;
    return spectral;
}


// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral){    
    smbus_reset(spectral->smbus);
}