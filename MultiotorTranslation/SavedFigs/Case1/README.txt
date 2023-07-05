Scenario: hover at 5m commanded to go to 10m and then at 4s commanded to go to 20m. 
Sampling time: 1s
CLTF: The CLTF is on the wrong system, 'sysCT_wrong'
Kalman filter weights: V = .0025I and W = 10I, so it is favoring the measurement more.
File: 'workspace.mat'

Fig 1: Plot of currently available states, 'available _states(3,k)', at time 'k' and estimated states, 'z_hat_l(3, k-d1-d2)', at 'k-d1-d2'. They match closely meaning the estimate is close. 
Fig 2: Plot of innovation, z_tilde_l(3,:) vs pred_time showing they are close to 0.
Fig 3: Plot of att the different predictors with the actual shifted data showing the KP is performing better than SP
Fig 4: Plot of estimated controller input vs actual controller input