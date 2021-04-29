function [Y,Xf,Af] = ANN_vert_60_200n1_normal(X,~,~)
%ANN_VERT_60_200N1_NORMAL neural network simulation function.
%
% Auto-generated by MATLAB, 27-May-2020 18:57:28.
% 
% [Y] = ANN_vert_60_200n1_normal(X,~,~) takes these arguments:
% 
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = 1xQ matrix, input #1 at timestep ts.
% 
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = 1xQ matrix, output #1 at timestep ts.
% 
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = -105928;
x1_step1.gain = 8.43401437999452e-06;
x1_step1.ymin = -1;

% Layer 1
b1 = [280.17161132306785021;277.1871840345101532;274.37187657858345347;271.55810546844088549;268.78114808934122948;265.92183184131505413;263.10908054477789619;260.31453255880882125;-257.43999999468934448;-254.64462439737980048;251.96541317868695842;248.94501455044371596;-246.4686436441624835;243.32157977065102727;240.64547199851756432;237.81278314486536374;-234.84821890349749651;-232.05961183946365622;-229.43216982290721262;226.65011978632966816;-223.70062418784581837;-220.88629767463297071;-218.09013231230383667;-215.32142785629520176;212.4967585809692423;-209.66583009116365588;206.78844641343798116;204.03687505942264124;-201.11911190101335478;198.36569691666710469;-195.57512119884933099;192.89428332955165502;-189.91666163258844335;187.1138734303105764;184.32647396311500643;181.35756636302653533;-178.81705383154269384;-175.89814470783147726;-173.17814045209073015;170.24626001804654152;-167.33843538919379057;164.72751095948211741;-161.79264233832583386;-159.08813118619747229;156.19221629960301811;-153.44649885487788765;-150.67898379845090062;-147.73542050171005258;-144.90369226936533664;142.13072692330169389;139.30060936869458033;-136.43696157459871188;-133.58924546779695675;-130.89619590134549298;128.14413550987123358;125.22625355423245708;-122.22426394577411202;-119.54920323344035182;-116.80096738074776397;-114.21904051201507002;111.0207027195869216;108.34754899118729554;-105.54328772911695467;-102.7589925838527023;-100.01058058266184503;-97.218357411314030969;94.160490697785704128;-91.731356447845598723;88.487513877395429063;-85.860583873371439267;82.991261772600140034;-80.107617352255871879;77.407558711267199669;-74.58996882565180897;-71.730591828869776805;-68.927676209237844773;66.292994224734897557;-63.296072512302764324;-60.438470477689399729;57.719867445556289454;54.781027300333875019;52.189953375202122743;-49.142248512100273672;-46.501697499863837493;43.689317680200673522;40.762436394519227179;-38.071656314103250907;-35.288975253476344562;32.247005763405468315;-29.617363853734072876;-26.753109509476349359;23.865578167888884309;21.066609839180333807;-18.326125970653432518;15.442099911330258166;-12.642672048639838067;9.7817859815506018606;7.1442204748357820421;4.2351089193936708455;-1.4091302512686081361;1.3832111724773568096;4.1008841911585429685;-7.0970137301201541291;9.8359874663354531776;12.690656519800308288;15.478228497202099589;-18.288069725247812158;21.099270531650322624;-23.869851351081251778;-26.675574104133755071;-29.625859503622589131;-32.411231230137353521;35.027832354004935667;38.004565922500589181;40.798458179331674955;43.523977955228815517;46.454021783372276388;-49.271833547197083192;-52.066685782772772484;-54.838641927878022386;57.538978183057210458;60.344673275059044215;-63.402303537669290279;66.107109181575467005;-68.950689289622886236;-71.814713661776821141;-74.530042368131248054;77.377393422866390438;-80.370281919681659133;-82.917653563402552663;-85.881794918728232346;88.61703090827334961;-91.479300043380320062;-94.285193521804757211;-97.105789858198150455;-99.872331777777546336;102.69982367873927842;105.68986321392092975;-108.12393473475680139;111.15420549145845541;-114.17536181629562009;-116.57716428463214697;119.63339345679712267;-122.4081157451567492;125.15598569061997125;127.96705487572556592;-130.74400988524465106;133.65289032815960013;-136.42936244696261383;139.42310506453765129;-142.00253928543344273;144.83806101800644228;-147.75008465560287618;150.49137036953720781;-153.37253282585928105;-156.1458373719247561;-159.08927046334318334;161.76547295382272296;-164.61331839987789749;167.47202512410132158;170.26207793372691413;173.04994529668007885;-175.90302907234425334;-178.58833033187622163;-181.48787773000279344;184.23231765748676025;187.26507566094485924;-189.64463289720728767;-192.79770482387624497;-195.60262056867500746;-198.4286304411526487;201.25358257116778304;204.17854325992368558;206.69186580268925013;-209.7936376130707572;212.42327148064333642;-215.27638368860320384;-218.14995934817520151;220.87386905823763072;-223.679163610453827;226.87043576350603757;-229.55345927915453785;231.89999134007544512;-234.54578883920578392;-237.72856789322577242;240.65340217011208779;243.15911441462998255;-246.31306753404359711;248.92763390851294503;-251.94908970303524143;254.60102363484767807;-257.7502228137060456;260.22603610511851002;263.1187073549493789;-265.87128277452006841;-268.71649546824335175;-271.35865828302496539;274.59179522746256907;-277.18119281316859315;280.09715478097922414];
IW1_1 = [-279.82827808994892393;-279.99874698576985566;-279.99998276105935702;-279.99969604803015955;-279.96407370295258943;-280.00742756496879338;-280.00606196220354605;-279.98769998019457717;280.04348646576386272;280.02519370736797555;-279.90441426810548364;-280.08896771255757585;279.79043942791673771;-280.08292317878385802;-279.96290351617500392;-279.97973860524143674;280.10572552022824766;280.08257666261818031;279.92871373492874909;-279.90394810851387319;280.0143247593807132;280.01398365789231093;279.99982726752142526;279.96511056999429456;-279.9737725741599661;279.98683077419678966;-280.03347046078908988;-279.98778972267439258;280.06197610219538774;-280.01854357597784428;280.00170334764266045;-279.90985253615457395;280.02248629751960607;-280.01411849728805237;-279.99639765463825825;-280.0962498786419701;279.92057586471156583;279.98781955891593043;279.93007266429043511;-280.00298681840195059;280.05880671478377053;-279.93839652369155147;280.00948621900232638;279.94725687224030253;-279.99365561337333475;279.95630600117345921;279.9316024402485823;280.00174100341234862;280.01063339992771262;-279.98970065143907959;-279.99793341209999653;280.02211235312756799;280.03720156274948749;279.97969190009183649;-279.95222127480786867;-279.99980470799471277;280.08155162080021228;280.01943671580488626;279.99162353298908101;279.89795372093146852;-280.05331750422834602;-279.99775232288681082;279.99411091764562798;279.9833419687353171;279.96040556815870559;279.95381454914092956;-280.03576865764142667;279.90942027625357014;-280.04884255736897103;279.99025215032457936;-280.00707629776297836;280.02652231474877453;-279.99405657866373076;279.99510566013105972;280.00645093705441013;280.00333793861926779;-279.96004700063053861;280.00457852311006945;280.01371864276870838;-279.99338177517529402;-280.01683480662160264;-279.97468543604776414;280.0179409196442748;279.9883046895720895;-279.98885453317831207;-280.0053407954540603;279.98675426035350711;279.98516454309230994;-280.0121495670279046;279.99181835922342998;279.99822979892923058;-280.00437008723315557;-280.00227869619311605;279.99680383308430009;-280.00191777045068875;280.00090760890009278;-280.00097475370722577;-279.99615112329001931;-279.99969889439233839;279.99998885032283624;279.99993481840152754;280.00173308233814851;-279.99816952807373127;280.00054765126100165;279.99785768341212133;280.00007680555302159;-279.99975740066156504;280.00017743697674177;-280.00413658255524751;-280.005599035338264;-279.99156859572258327;-279.99434015005522269;280.01831493861504896;279.99764023648668854;280.00027746174623644;280.01390558044056434;279.99628764611304632;-279.9952323247588879;-279.99876220012453132;-280.00623386082315847;280.02960866586386146;280.03317828210134621;-279.97994112728775917;280.00502769284628357;-279.99820238384864979;-279.98568956314426259;-280.01071983849720937;280.00270552403202373;-279.95059232210508071;-280.02751955808190587;-279.9838365873332009;280.00751448260314191;-279.9929388424247918;-279.9953615049054747;-279.99295164928531676;-280.00966901225774563;280.00501546896890659;279.93840233691679487;-280.08341938397654758;280.00066735658094785;-279.91514628539897558;-280.08534884861694536;279.98484101683504832;-280.00171791656322284;280.03077980935330515;280.03281973905035329;-280.04973091381435779;280.00693874769990543;-280.0258616696683589;279.93613223087254482;-280.05381304285589295;280.04448234025545617;-279.99358620617027782;280.03285905597829242;-279.99677641454650256;-280.0195730162088239;-279.94593084872548161;280.0250520752946386;-280.00554528072444782;279.97896655679767264;279.99297493397887138;280.00950452809900071;-279.98487126920946366;-280.06679442502252186;-280.01263029426723961;280.05852868520656784;279.91316666101130295;-280.20626390213516288;-279.97658385178749541;-279.98272262669286192;-279.97392640322658508;279.9649927048454856;279.88395723766495848;280.10462605368945788;-279.88976463080024359;280.02828023868471519;-279.99993211152974482;-279.95344324037660044;280.0241746073553486;-280.03119909557153733;279.72616989343094929;-279.82988629815889681;280.21638554438357005;-280.35915643305338563;-280.0510241894546084;279.95646523478910694;280.22418014207107717;-279.9277905140893381;280.10421437928556543;-279.91889191702898643;280.06570102134890021;-279.75723790221923082;280.06983063566593728;279.99703883886462563;-280.05521898898666677;-280.02598313397260199;-280.1932875990953562;279.78459946223472343;-280.0046666998763385;279.90284543923041838];

% Layer 2
b2 = 0.72161819871635446422;
LW2_1 = [-0.25112868764522944254 -0.059529070830387588453 0.12961599911370946803 -0.024350812023030105058 -0.001303239406769851616 -0.080438490591105665795 0.093510436188056336726 -0.01977574324262693084 0.01449540760252852889 -0.062341012413643093204 -0.050846697442762600616 -0.015361436895216240561 -0.020924685148829780257 0.0032362289376429431709 -0.0057589024683793177406 -0.0040559040199573993812 -0.014027921959157113596 0.049919556349335869605 -0.049887735498473528839 -0.012724787602904602535 -0.0023500514075545167469 -0.0036086492809749015631 0.0034162626815456564877 0.0088077434654055541952 0.0097249541502573977336 0.015738410109404576964 0.0097472507800122065724 -0.040383463473677216893 -0.00368906479084243899 0.0035845295040546343301 0.049455407538831236414 0.015097645682364699282 0.020553794794336222029 0.015650603553639712978 -0.011488224323714747721 -0.0020025944329721823145 -0.013797490733278960917 0.016505088108099030819 -0.001666838494338178378 -0.0021968853341536170865 0.0030970675573051692497 -0.013669285647692611119 -0.024835542252244026395 0.094213873446743562345 0.02456741784939302925 -0.00094691801726974611508 0.035085306252468320687 0.045132887517250308562 -0.033621770756410374048 -0.012838691930745486924 -0.0075965673009415245545 -0.0049040074527216433176 0.0037743432476519817032 0.0015347711250215945178 0.0020687673547295920562 -0.0034781733554621260818 -0.0033155152828701073275 0.016676775387955811158 0.0045839982682283064719 0.016900118741666858568 0.0059809463232891497964 -0.01496437681243884335 -0.0017954302112262142399 0.013417669062042954231 -0.012521929985456385984 0.024665781023332663396 -0.0023055406908286807388 -0.0018424384612811504913 -0.0033518832614020076244 0.0091385980256793938881 -0.0088610551409594590089 -0.0087239816881299683815 -0.0039180290335934179341 0.007579728471738425219 0.0037129122820352290085 0.0004524730893412497873 -0.003974080005169623904 0.012676898292622442582 -0.0019893049180596087018 -0.0077882266462840884017 0.0011972486224925056077 -0.0048125246057786601889 0.0075665668603723292007 0.002635870772237507674 -0.011477355665024112016 0.0041893577747875313314 0.0087180848688565276 0.0042455478176831222895 6.4850240137823665643e-06 0.0098355762390238098136 0.0085040027951087619057 0.00026347221704934513202 -0.0052712733982851348383 0.0067774462791865466155 -0.0086898833625106298301 0.0010675732910407677041 -0.0013241965403104081003 -0.012215073073670836695 -0.0014831818136151289113 0.0040429941266706698882 0.0045067220677129310805 0.0065530401335091347306 -0.0026424827485865445657 0.0099846063843847861902 0.0042777638870150928516 0.0046778623159216010247 -0.0064554441361848553457 0.0090331907730698181996 -0.013221746727246347711 -0.006377232247795912301 -0.012449106056168680531 -0.0069600015528266745771 -0.0067066650378140978822 0.018543010565696602804 -0.010011081463715492887 0.017101735910909465765 -0.01296395058265021466 -0.01151260681455039403 0.0014377326420347766311 0.0044410665759649012307 0.01182429444012524436 -0.0086605279478096496709 -0.01043931354356969661 0.0052837644911213552112 0.0043300360761707561152 -0.011030086999225559644 -0.0063480850730522654646 -0.0062455088790029852919 -0.01048041930332557943 0.003740191863540453035 -0.0059713187997862812381 0.0075121897414521885544 -0.0059321809403786833481 -0.0086712325725391056069 -0.0017059288439379223686 -0.0031588198397638498141 0.00976528423912315785 -0.00011419221282077656604 -0.01557270893907898579 0.009179947235040588488 -0.00057477989958447037178 -0.0186287660392580115 -0.0030329748085131660526 -0.0056479328868639950417 0.0010844754150504225254 0.012329585956248357084 -0.0047251824856638915615 0.004389944301680683697 -0.049091647583540690136 -0.039262186728268498592 0.001101163115941640458 0.014184784313905839451 -0.0031847528367079987882 -0.015115361912513673032 -0.0088691936636970773733 0.0048462428478514719271 0.017219782889746949178 0.009860771721215542962 0.0059399495211358392666 -0.00011199891347879464831 0.0057721366912629190615 -0.017872577448487188961 -0.023173576167965399819 0.013894579705113756229 0.0029564518698947869768 0.014212968728703053931 -0.0062107984123782461472 -0.051249307260032059275 0.02262429580408061372 -0.034133993400475320934 -0.0061316151500994814372 0.010849430984031641079 0.054381303390493990202 -0.046141835608253176093 -0.076156342134479859673 -0.055734584690863619993 -0.03705599004670151464 -0.010854734371117016889 0.0049048634009538011683 0.001506628300559717179 -0.011934225787232091709 -0.10996213314192622434 -0.074966043998582393426 -0.055779367329394320685 -0.036297371999170366386 0.0063813064562307023064 -0.025405830799203284187 -0.055898752737079102082 -0.033017965707809987519 -0.04368575294910298501 -0.0058443496943108054487 0.020730456227466775854 0.027865078385316516263 -0.066230486468044469373 -0.02266217331259218204 -0.2089250749898685644 0.18453601364171279275 -0.23480004371079099545 -0.59442819537465874102 -0.85674441738024664783];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 0.0304481005460618;
y1_step1.xoffset = -35.3891977430646;

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
