#include "palettes.h"

// Initialize the current palette pointer
const TProgmemRGBPalette16* currentPalette_p = &GreenAuroraColors_p;

void SetNewPalette(int _palcount)
{  
  switch (_palcount)
  {    
  case 0:    
    currentPalette_p = &GreenAuroraColors_p;
    break;
  case 1:
    currentPalette_p = &WoodFireColors_p;
    break;
  case 2:
    currentPalette_p = &NormalFire_p;
    break;
  case 3:
    currentPalette_p = &NormalFire2_p;
    break;
  case 4:
    currentPalette_p = &LithiumFireColors_p;
    break;
  case 5:
    currentPalette_p = &SodiumFireColors_p;
    break;
  case 6:
    currentPalette_p = &CopperFireColors_p;
    break;
  case 7:
    currentPalette_p = &ZAlcoholFireColors_p;
    break;
  case 8:
    currentPalette_p = &ZRubidiumFireColors_p;
    break;
  case 9:
    currentPalette_p = &PartyColors_p;
    break;
  case 10:
    currentPalette_p = &CloudColors_p;
    break;
  case 11:
    currentPalette_p = &LavaColors_p;
    break;
  case 12:
    currentPalette_p = &OceanColors_p;
    break;
  case 13:
    currentPalette_p = &ForestColors_p;
    break;
  case 14:
    currentPalette_p = &RainbowColors_p;
    break;
  case 15:
    currentPalette_p = &RainbowStripeColors_p;
    break; 
  case 16:
  currentPalette_p = &darkishColors_p;
  break;
  case 17:
  currentPalette_p = &sixteen1Colors_p;
  break;
  case 18:
  currentPalette_p = &sixteen2Colors_p;
  break;
  case 19:
  currentPalette_p = &sixteen3Colors_p;
  break;
  case 20:
  currentPalette_p = &sixteen4Colors_p;
  break;
  case 21:
  currentPalette_p = &sixteen5Colors_p;
  break;  
  case 22:
  currentPalette_p = &sixteen6Colors_p;
  break;
  case 23:
  currentPalette_p = &sixteen7Colors_p;
  break;
  case 24:
  currentPalette_p = &sixteen8Colors_p;
  break;

  default:
    currentPalette_p = &GreenAuroraColors_p;
    break;
  }
}
