function [y1] = ANN_60_5n_normal_coder(x1)
%ANN_60_5N_NORMAL_CODER neural network simulation function.
%
% Auto-generated by MATLAB, 13-May-2020 22:29:34.
% 
% [y1] = ANN_60_5n_normal_coder(x1) takes these arguments:
%   x = 4xQ matrix, input #1
% and returns:
%   y = 3xQ matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-166881;627;-105928;629];
x1_step1.gain = [6.4553820133691e-06;0.000655522779416585;8.43401437999452e-06;0.000655737704918033];
x1_step1.ymin = -1;

% Layer 1
b1 = [-0.81937975946066354815;0.33527452679228103571;-0.77344523499712236614;0.70090676203811719258;0.66578434496813820864];
IW1_1 = [0.35161759622822769122 0.10343249420372759018 0.10081356909927442767 0.57481436436849486782;-0.91716425798623180121 0.87384646066643323348 0.028963260357590184246 -0.52474963543588293291;-0.047580147556815305387 0.057289887240155684678 -0.11924686106672328922 0.50778265093757002369;0.38270641259304177817 0.33482996171446549605 -0.37351064360996927283 -1.1141007115957259899;1.1443595848453784747 -0.43301268805111609028 0.0086078374082482998764 0.80975826209739110162];

% Layer 2
b2 = [-0.14488575665154293515;-0.7309050613653592654;4.4933661730823351022];
LW2_1 = [-2.8760553708548557417 0.018316730278642917584 1.393286384817246093 -1.3740602734090110459 -0.099068692837944893892;0.6809865588613389642 1.7987315294510721753 0.49237054277474723296 0.36269648547796728977 1.6197857106054560639;-5.4305906330005218052 -1.5929550466538746623 15.515732542219769385 5.0924526911956444764 -1.4046070339395220206];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [0.533787165877804;0.788527964418997;2.25106301047751];
y1_step1.xoffset = [-1.82517310309568;0.576613863304083;-0.470200245018933];

% ===== SIMULATION ========

% Dimensions
Q = size(x1,2); % samples

% Input 1
xp1 = mapminmax_apply(x1,x1_step1);

% Layer 1
a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*xp1);

% Layer 2
a2 = repmat(b2,1,Q) + LW2_1*a1;

% Output 1
y1 = mapminmax_reverse(a2,y1_step1);
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
  y = bsxfun(@minus,x,settings.xoffset);
  y = bsxfun(@times,y,settings.gain);
  y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
  a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
  x = bsxfun(@minus,y,settings.ymin);
  x = bsxfun(@rdivide,x,settings.gain);
  x = bsxfun(@plus,x,settings.xoffset);
end