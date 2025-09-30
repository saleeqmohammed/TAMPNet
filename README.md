# TAMPNet
Manipulator robot learns and plans tasks and motion based on vision sensor to solve puzzles

## BaseLine
Breadth First search / classical search as baseline / label

## Final Target
Lear from all connected instances 

## Nerual network structure (expected)
 CNN -->Action selection vector predicted
 CNN → Fusion → Repeat vector as initial input to LSTM → Predict action at each step → Feed predicted action to next timestep
