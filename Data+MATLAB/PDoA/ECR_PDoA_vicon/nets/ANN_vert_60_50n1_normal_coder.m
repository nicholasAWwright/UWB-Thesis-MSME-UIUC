function [y1] = ANN_vert_60_50n1_normal_coder(x1)
%ANN_VERT_60_50N1_NORMAL_CODER neural network simulation function.
%
% Auto-generated by MATLAB, 27-May-2020 18:57:58.
% 
% [y1] = ANN_vert_60_50n1_normal_coder(x1) takes these arguments:
%   x = 1xQ matrix, input #1
% and returns:
%   y = 1xQ matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = -105928;
x1_step1.gain = 8.43401437999452e-06;
x1_step1.ymin = -1;

% Layer 1
b1 = [-70.477358442375617642;-67.121121084482794572;64.295678663743089487;-61.443831516473302656;-58.480453509603187001;-54.330913950872066209;-52.409905207682406569;-51.092649818066178113;-48.153285660531338408;-43.962154718389726327;-40.529971630208329714;-37.374635347234978155;36.653999338399643193;-32.882525268214834568;29.711377639822838859;-26.832306671498741935;-24.197812726163295594;-21.710571110574225173;18.182542153661795936;-15.692982720055182355;12.35741385098158851;9.8526300278353300399;7.0882655275671595518;4.1670545596027315227;-1.5567579677035150443;-2.4211088506561875278;5.7262503069999954874;7.2318644319218607919;-9.6388132006503006721;-12.871875646631881551;16.115293562723692844;-18.392636184340467764;21.901828224944544843;24.507196483360186079;-27.391203078617301969;-29.047370003268600414;32.411570637858396537;-36.924572075875055077;-37.760080273303607612;40.71624723971845583;45.942212642163319458;46.943702527430453131;50.041870289520957726;54.022155964280244689;57.189170184352036586;59.145835960025920031;-62.331360634730671677;-65.380504181420420196;66.849788623041590085;70.820367880275000516];
IW1_1 = [69.52278168095246258;70.020372578011972564;-69.990559973272056027;69.985571463808426529;70.075279435871919986;71.092122144652748261;70.322021234229282527;69.290993321171455932;69.349869563293438546;70.230681500470836909;70.895532469902136086;70.733121623661929789;-69.658446045893057885;69.979957532217056837;-70.162207702305963153;70.192619924398570674;70.210973704324274536;70.050111482443270461;-70.202013532221670289;70.044547617617993751;-70.132134517151385467;-70.061451447245133295;-70.698173002024645939;-70.367811507048187991;70.364880091161538189;-69.908452692658840988;70.573723562857821889;70.240424276990751196;-70.664817696646125;-70.179644834660592778;70.164771892627328498;-70.434075165071405422;69.919622554159218453;70.008604630791651857;-70.186991322153062356;-70.692120063630795812;69.573230334275066866;-69.414896517068186199;-70.872993207937170723;70.459816797406233491;69.042694528163735868;70.313558532587109084;70.182148468552966847;69.898839138255468129;69.783685637300749249;70.211834349196180938;-69.372987703663682169;-69.040307246512213624;70.400884413941497542;69.192527923327830308];

% Layer 2
b2 = -0.51984430692935457152;
LW2_1 = [-0.002020830429065624545 0.00028226992339971203111 0.00021781311017279975334 0.00034777265845997121663 -0.0010874250882440779972 0.029800884147026109744 -0.11381646727435644484 0.13847451470719021605 0.035579188184588092514 0.0024643169228500804574 0.054575290432021562881 0.33858520552592613617 0.27059105813642558847 0.010708424077028679683 -0.029184219095917455361 0.019513812984077782975 0.021503890622404233518 0.012828032993148680979 -0.016487296122889866928 0.015893835159273211194 -0.021798545807547000525 -0.013635022403776363153 -0.021595320718407841343 -0.019998827636040574646 0.021921113534287318264 -0.027314734877375769495 0.034011978128994785342 0.019445142055607263915 -0.015495629820018189116 -0.0056807756218149076111 0.012422901816870311059 -0.012538870223506498602 0.01995538378426721729 0.023131456639086681037 -0.029604692809823911215 -0.016245001009912357626 0.031045773424344880881 -0.622939971202938092 0.61809554476866124251 -0.010214215240851405239 -0.2408335718636270828 0.28286948728507466111 0.073854008205252313912 0.042933379071712525488 0.065066088176722713943 0.0593923804148256268 -0.046737599170909395474 0.49406366128688811168 0.48996094437511783504 0.51123891800330956148];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 0.0304481005460618;
y1_step1.xoffset = -35.3891977430646;

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
