function [Y,Xf,Af] = ANN_60_30n(X,~,~)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 02-May-2020 00:40:58.
%
% [Y] = myNeuralNetworkFunction(X,~,~) takes these arguments:
%
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = Qx4 matrix, input #1 at timestep ts.
%
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = Qx3 matrix, output #1 at timestep ts.
%
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-166881;627;-105928;629];
x1_step1.gain = [6.4553820133691e-06;0.000655522779416585;8.43401437999452e-06;0.000655737704918033];
x1_step1.ymin = -1;

% Layer 1
b1 = [-12.113629720601245765;-3.5536559185501461755;22.048525752000429634;2.3811264466876203016;-9.5896756250093790896;-2.1679853034735421247;-6.8468456796730094993;-1.2017126985056982846;-0.40003349491739215171;-4.3051517225277722289;5.310703297362642239;-1.2419758967659009219;-2.2982268889462167394;-1.0410294695801149967;-8.7311923217673452768;-12.057790564580143311;-0.63691707052097912278;2.1249151255609324274;2.1387953821925389519;1.2437190016160584172;1.5709213206018255438;-5.5906195380141339157;-2.8803669660087378723;19.582049597637976035;-1.718369189788360174;2.4088251991834828125;16.19072590490247876;14.433424034596294305;-23.351154621816196055;-10.146277271005557097];
IW1_1 = [13.337482586633889525 -1.2514058251310520298 0.23180007536200050722 2.4271579233912916607;4.3162710065509219959 1.4178197231352729357 -0.012301327623030032868 1.5923436077958754709;-3.76982920906705532 -16.876321209779717947 -0.86340936026690873195 -6.3108561252413650777;-1.9546731300506638984 0.81401272425138315736 -2.6391751012481532435 1.5641366647372210164;11.527934507894979532 -12.443066937662681681 -7.5408120359636736652 12.676905295662574957;1.879342606161031437 -2.6355997534002977289 -0.34135893506443298895 -0.59028100099871427808;0.14416646287285719841 -7.191419453851045418 -0.37531905609728860584 -3.1146218491624555469;-0.38027368825653018458 1.041227230940583004 0.18305083480041514643 1.0710786693834699612;0.090041854854213468351 -0.39270460277831886753 4.1645259187515089039 1.3847196018672751627;2.7406473529490735785 6.2050870064636569623 11.830909368534539894 0.35147769712007764786;1.5934369306044036119 4.9311475984027479669 -8.3722619379512259741 -9.3810744326204815735;-0.79475725954292053554 1.2784790357757467305 0.31680663158043648187 1.2385070783144904727;1.9327744503472974458 -0.79307722195420915501 2.6302156613601477275 -1.482010182308158841;-0.60117200578985252601 -2.6780384908171779301 -3.4129236138347791574 3.604004415177632481;-7.5143868985309891784 -17.983975211644100511 -0.11433288588466532232 -12.713212380257182588;79.919662773100256459 -15.619782935094431053 1.3091458524348031922 18.33716638991355552;3.1933162518939126606 9.1744795725840084799 0.35495249472122814005 4.1330496887642311776;-1.9962804910470395292 2.5605712126313422417 0.35578725031032626402 0.58838845023441177506;-8.3916030288550569338 2.193991906136540937 -0.71326057122100194885 -1.4448776885258465263;0.98819112492539262771 -1.2622006923546740254 -0.3870771061959913939 -1.3829462717746956457;0.49266083938323373959 3.2648084340534788694 0.45135475580068251666 1.135772347012863559;-44.823690790132921791 2.3804652962493828561 3.6784048574657872699 1.4540103502192904816;-6.5648737060261641574 5.2568597493889477335 0.028000597418312803133 1.4768694519595335635;25.431771676149562467 -10.893979511125456483 -5.4550807392930042994 1.8562070669264367773;-0.51964219379268039578 -3.5476045989264348712 -0.54228847433845417392 -1.3262714020695887118;5.4700532823989389541 -4.6083216682940735254 -0.027832806857128685601 -2.5124470688342204205;32.913139612493615971 -15.822959918589576844 -6.9137044323520155942 7.6366273104522415238;11.096302424705202228 -16.35074260933344803 -8.2642857763918087244 0.55040908500485141719;-27.866655953404595891 20.495581143198680962 8.8854436612139391372 -8.0005790218914647483;-10.025603841205196431 0.93884191671432559723 2.0797987744353969575 -5.4587357561029739017];

% Layer 2
b2 = [-0.12299216894816066004;0.093382744693107366807;-0.25736602638205036131];
LW2_1 = [0.202513940013757332 -0.32531754297129239495 0.094368399814701520167 -1.6715927982906486626 -0.014823099221976129489 2.541887070469003973 -0.018030219779426856497 -0.84368063376575230805 0.0052539444591607085619 -0.012693444608837941776 -0.055910224709456488035 1.44461478896379103 -1.6975998608518982902 0.014967974756700455977 0.093245581773393398928 -0.12556699909837226725 -0.053824864735203092891 2.5022110877458096923 0.010155230412915336805 0.66275149887747464295 0.56598565510230891551 0.1375667763252841902 -0.15996849665163767606 -0.17727259091225372467 0.39127014848180730056 -0.15308517879266617112 -0.13391773436578732759 -0.025494806446732243277 -0.048009460564417683059 -0.12462072483178550375;0.24897550097340750974 -0.29121187088542382915 0.065592396920890139178 -4.2636555168147047823 -0.040359654683403183018 5.5817986871248814751 -0.5064798774039384055 1.1525312581193307704 0.038305695274518519744 -0.031130147413871604029 0.014993740050088235835 1.9115172691989466003 -4.3413022158945544504 -0.023503116427170300112 -0.1933090170027067789 -0.022207532877717015701 0.13425806125132400126 5.3461853301672759287 -0.078226949667582645898 2.3313252608864392101 -1.2807297226497371678 -0.033727678541909777987 -0.117636347441328587 0.2219699834309168951 -1.2436180085829182662 -0.22363361147738000656 0.042879197031030373555 0.08547259760140987761 0.079110208866470299593 0.14039598405377437529;0.07698067855682821925 -0.099747836330602496213 -0.055117441884078799408 -0.83107782048581979595 0.081507416097210405592 -4.7818242115820721594 0.52617796157325458228 -2.500647852036236074 -0.62743291555993019326 0.0075507922728214335942 0.2703964580567565501 11.512793846048237967 -0.6715223917046774238 0.53664637882471555308 0.076129992211919586542 -0.1533116852530242713 -0.13848574026031143513 -4.2920417241328143731 -0.11685382599020029126 9.2302428311270006134 8.0513016868235656176 -0.039894195385237564422 -0.1101945292085408229 0.68762313731569846276 7.5455657905239847949 -0.24219434996423769291 -0.18844179698125068656 0.24877686933650869361 0.88705263953957325107 -0.26444780293979863739];

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
    Q = size(X{1},1); % samples/series
else
    Q = 0;
end

% Allocate Outputs
Y = cell(1,TS);

% Time loop
for ts=1:TS
    
    % Input 1
    X{1,ts} = X{1,ts}';
    Xp1 = mapminmax_apply(X{1,ts},x1_step1);
    
    % Layer 1
    a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*Xp1);
    
    % Layer 2
    a2 = repmat(b2,1,Q) + LW2_1*a1;
    
    % Output 1
    Y{1,ts} = mapminmax_reverse(a2,y1_step1);
    Y{1,ts} = Y{1,ts}';
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