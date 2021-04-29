function [Y,Xf,Af] = ANN_60_10n1_normal_compiler(X,~,~)
%ANN_60_10N1_NORMAL_COMPILER neural network simulation function.
%
% Auto-generated by MATLAB, 13-May-2020 22:33:40.
% 
% [Y] = ANN_60_10n1_normal_compiler(X,~,~) takes these arguments:
% 
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = 4xQ matrix, input #1 at timestep ts.
% 
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = 3xQ matrix, output #1 at timestep ts.
% 
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-166881;627;-105928;629];
x1_step1.gain = [6.4553820133691e-06;0.000655522779416585;8.43401437999452e-06;0.000655737704918033];
x1_step1.ymin = -1;

% Layer 1
b1 = [-1.0441030906434212167;-2.1295972516822971876;0.41328116917304114075;4.1793077190618994621;0.32503738699762585451;-0.75433318335734267368;0.97854127917052891039;-11.405860199383106135;3.2565757585608499092;11.267959100140558348];
IW1_1 = [0.052485775172202528416 1.8640702274249947301 -1.597983559443259205 -1.9185434366251525873;2.9816286391823392243 -0.77814680674671876659 -0.027870252242504535423 0.32892727676546862359;-0.14865000917068510455 -0.2377832237260568693 0.10621332014543879985 -0.067035017966997831329;-5.5855520829872551403 -0.42689078342642344799 -0.084870458137734763882 -1.7622156875456123082;-1.1154719472661458468 -0.56402021345796438112 0.025041180024450424013 0.022163774687553920717;-0.024597122526852865931 0.13163873842277989956 -1.2141930670822067562 0.024653763019392302414;0.15399221888704969774 3.0050527069523229606 1.2662566113465205664 -3.1006720348281708866;-15.107003098383700035 -2.4487036461332780668 7.9708189417224826201 1.1531387024432897537;4.8384958607617472026 0.56465776762208297335 -0.70668846164423049228 -1.731066722079473541;15.113407121427250956 3.8410492198295456134 -7.9333258576161647113 -2.6473687338326579166];

% Layer 2
b2 = [0.93113982387559557363;1.2554166720427297754;-1.7113961174363512274];
LW2_1 = [0.4030070650624351658 0.39122671037258843763 -3.278891441413489094 0.33234294462156377392 1.5831628241419715142 -0.89286516655770031825 -0.16868110735902189234 -0.2163218439868429821 -0.17782512172611811274 -0.22941264924443377504;0.035390231976414950055 -0.12698105025062692008 -6.3083879122447470422 0.31627524015029856441 0.76777392101855013173 -0.86856004909659956237 -0.14622582062032501993 0.34231268194822517259 0.30116866293922278652 0.33265210687567414727;-6.1086071703848512371 -0.077040621831135483455 7.6127134612546552006 -0.16983223815016626479 -1.0832084581061935946 12.958262898162269394 3.6295476020917041282 3.3750896674567636424 -0.3287751404634751129 3.4826964344893602821];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [0.533787165877804;0.788527964418997;2.25106301047751];
y1_step1.xoffset = [-1.82517310309568;0.576613863304083;-0.470200245018933];

% ===== SIMULATION ========

% Format Input Arguments
isCellX = iscell(X);
if ~isCellX
  X = {X};
end

% Dimensions
TS = size(X,2); % timesteps
if ~isempty(X)
  Q = size(X{1},2); % samples/series
else
  Q = 0;
end

% Allocate Outputs
Y = cell(1,TS);

% Time loop
for ts=1:TS

    % Input 1
    Xp1 = mapminmax_apply(X{1,ts},x1_step1);
    
    % Layer 1
    a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*Xp1);
    
    % Layer 2
    a2 = repmat(b2,1,Q) + LW2_1*a1;
    
    % Output 1
    Y{1,ts} = mapminmax_reverse(a2,y1_step1);
end

% Final Delay States
Xf = cell(1,0);
Af = cell(2,0);

% Format Output Arguments
if ~isCellX
  Y = cell2mat(Y);
end
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