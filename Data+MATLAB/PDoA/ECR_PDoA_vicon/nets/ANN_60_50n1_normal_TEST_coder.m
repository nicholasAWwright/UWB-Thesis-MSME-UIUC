function [y1] = ANN_60_50n1_normal_TEST_coder(x1)
%ANN_60_50N1_NORMAL_TEST_CODER neural network simulation function.
%
% Auto-generated by MATLAB, 20-May-2020 20:28:16.
% 
% [y1] = ANN_60_50n1_normal_TEST_coder(x1) takes these arguments:
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
b1 = [3.3216088585854977033;5.1822192311837609324;-11.527035024077157743;-3.7517566193578302247;-6.4832753606700190829;7.8666848458349001305;-1.4100928416102220808;-12.281499060944433666;5.7832398437457666063;-6.1238335858243138077;2.249843925213523832;3.6495274115369924495;-2.5703536182478687877;-0.021184459993929882077;-8.8115866005902887537;-4.58103517566704177;0.057918149332534224483;0.0044774229286811319944;-3.5417255553986919381;-1.2460944934116711114;-0.99021602935051544758;-0.061569457756926106173;7.705941302778119173;0.77607178622693517855;-0.93425894723570623857;0.82119592904453020399;-0.90434570579221063014;-0.98878454183887454221;3.4898523440893467473;-4.6495378324693223959;10.043205521230541777;3.0614365690047531388;-5.8089113009595134685;3.8641749704694223766;-0.88438158651366738372;-7.0014807019713565239;-9.5974631937965497741;6.1942554990828373462;10.150041580361920168;-9.3559036184921513524;-0.52179350196399498785;-2.879898087849301902;5.8370916243033770243;2.7436137672637368468;13.958017820522529817;-2.1423188807979784976;6.6223509818060550813;1.0096495821188622433;-4.7515303894661933271;-11.112439100582379936];
IW1_1 = [-4.1023073328014447725 0.96934432207204079823 -0.61463680107482487891 0.70516631855737665902;0.30433284534324256398 0.54557201840307267826 -11.781942615034550315 -2.8691617180195048142;-5.9614849264835427789 6.2503416091670667143 12.53876458801774163 6.3000767778158861532;4.2010088986670153233 -1.0719099241677518819 0.63552596817256912232 -1.0876587533354102888;0.72878263256864883601 -4.2465518482866855265 0.68651169501075293145 -6.6033510185365473788;-11.523600513501358833 3.7954180045216188155 -0.55183283154708484552 3.9252023752908087317;3.2665070433365386293 0.71876211859976801843 -5.1931243863127845017 -5.8510928218347499907;16.233431866523311271 -7.1310272496427220545 -3.6248893277523208667 -4.4512595610585412587;-0.43595432647891591005 2.2682653697619898914 13.128970638218905975 -4.3970617094952757498;0.35396912028823401819 -2.6581750137024062752 -13.965814959814093044 4.8898041833296854719;-3.2174263921851617987 -0.29213971342198052916 2.4479351226495493776 0.54483884659337733503;-2.0697340081875319484 7.3837153311102055042 5.5990575948470961976 13.640771758857965779;3.6438355839167329009 0.43735640865956643131 0.12209580855661926468 1.102675244966203838;1.991831661177274837 -0.47236062982871274141 -1.4196265210868947992 -1.1026080838808602902;12.847992544573385132 -4.628440673261308369 0.64329266609601976068 -4.0191565313285080308;0.66795870656027633849 4.7494232464071606259 -3.3560859619073828419 5.5124285172666382948;-1.7368256336796610295 -2.542129623548016859 5.0552077374381134689 -1.1494635574388611943;1.6436274189509774146 2.6331268725851262325 -5.2921515077585832287 1.3040088920599306466;16.804777520778277022 -14.107800318194270162 8.2936551292189584217 -6.3227533507850104399;-1.5514789951828960035 0.8308767101405302169 5.9232321776564020865 2.8819981218516605992;0.7308894372834092712 -3.7242793072591351411 4.4375072246521716224 -1.9647526875206293706;2.3802620565018961507 -0.58087971241200153649 -1.7582235558855661139 -1.0093171863875056715;-16.177696910822202625 1.0180515978292181689 -16.79936096684591007 -7.2114534462380106561;-0.42887218317743469598 3.2884770558561031706 -4.0168237963671824531 1.6456879513849935748;-5.3003252308228558931 -0.42300601971349360841 2.3378592561390401094 2.6537955159696640095;9.2904609670034670188 -0.605078564182733869 2.6362591673820143789 -2.3155039348298798174;-6.2222379189115688547 -0.14130134324048654326 1.5859974461364274578 0.0024051515044369517482;-0.62364949218924736751 0.89488038948344683643 1.8607131443417723737 3.49463598537614617;-19.330696990628144505 2.0628251609759087337 1.1008516661623874988 -3.9218431540044389472;-8.9330817863595584072 13.235843081336238214 -14.613057269494627022 12.758487085802569183;9.8640317362575657967 -8.7633527006381992663 -16.829430951480485845 -8.8304175899622094903;2.3206042951286804943 0.64517270527552272874 4.562732935170330073 -2.1652891611367719449;-5.1888555105092626363 4.7059853299235756552 8.2399568205132087684 5.6792915872142426537;8.5474630442813968045 -0.29611301792140376499 -4.0400076217435367809 1.8327823861114320714;-7.7997494580875397219 -5.2293494379550393703 -4.0149143446548967162 -0.082549963427720932096;-0.5327153078384744278 3.0477774844883329664 25.940738275501153964 2.7940701968712677861;-14.127274927102256186 -1.8610967308636074957 1.7806788971445983716 -9.1640655739228193966;11.230623671041850997 0.14406230981517506584 2.2792594017452820765 -3.7360262895624374302;14.443083142416250908 1.2212046573280190831 -2.1585406524974315445 10.599367664595396477;-12.347300784546197505 3.6362903638408230478 -4.0422103957258705975 3.2910802778329681928;0.3214576765929912372 0.40658145407157980644 -0.11417213504138867819 0.45451992447295386235;-1.7499073275601770838 -0.12469609330232288291 -4.251916275786643773 1.6679453250895530658;4.8065763822629365976 -3.3032929543679263595 -5.753424581033282692 -1.7682871321362039208;2.6445922222337547858 -0.80633821536204264291 -1.9103054836101347469 -1.0937251083979973032;14.667890590385010086 3.7298081652111916284 -8.6142692126710329603 3.751577484045748978;-1.2921208836657478436 -0.29748887541801849466 -1.2422302995560645655 0.18875500688587068532;0.20165930434527595816 1.4318269800093628152 -9.6977817583979710747 0.12939320670084517029;0.43193603287113713218 -0.18056439701742965642 0.7092151369915190573 -0.47010282507356371262;-4.393762347531496637 2.3275482091931336548 3.7275149203363580064 1.3855661619918795413;-15.407233909070161104 -0.72198685455351818696 2.5314604985926227876 -12.314965252697140485];

% Layer 2
b2 = [-0.0064115360169357477407;-0.098080499141566981902;-6.1802306918372815758];
LW2_1 = [0.10810431699858494281 -0.014077956665576525294 -0.094085071606022446788 0.14752884542674421509 -0.022287327161010239002 -0.79734147019022705294 0.036676431389267204197 -0.018256058011900892085 0.28384852738339610978 0.26304725200212869529 -0.014976775226281323003 0.020418401508572991965 -0.31164804035701676455 -0.6618942724599159444 -0.77891471026574288228 0.054498416831354618006 0.04982695951998412226 0.061802907976268882384 0.002087133301340083659 -0.0015593954232597435385 0.0019502337392693284805 0.49380834140657325904 0.0020588605276542666644 0.020048766967693393282 0.0016855792127228480005 -0.0048014506492727237683 0.088176720452066301159 0.020657078996002154608 0.10238495195848994201 0.00082546099420737904968 -0.040332510832974843851 -0.14693064281002404803 -0.060728246025693634336 -0.034891102195708623146 0.0013398758627416016039 0.006489492437469020561 0.25948322778460802818 -0.035571079531606944468 0.44485777500270745621 0.028668003020725658231 -0.60949683453827419033 -0.23505914323131277266 -0.17913863318445644168 -0.41545168619065259152 0.010845599607355534827 -0.090904500090444650962 0.04604700474302284019 -0.4562357654657910877 -0.20789892527521633414 0.1826235331456996791;0.9339774378420468981 -0.028442923360900755625 0.1067285271507218275 0.9232020182440258127 -0.20249227135931407995 -1.9759971870041381781 0.023640877009160260469 -0.041822086995139694243 0.61140218803987600449 0.57830811192643261087 0.053968339677661486542 -0.019510366602859263718 -0.41161306155964788589 -0.36401204898611305705 -1.9167570560713567129 -0.032190001332057981143 -0.21808497367082627094 -0.21560941710299758034 -0.0073842334499177033832 -0.043163995394611805179 0.075254986790567607313 0.23215245099847192645 -0.0062001627974540346119 0.062013836944830008635 -0.014027868064066903453 0.051799037863927352432 0.13689800540583993427 0.030504547633683095492 0.062315079940252891821 0.003622655664726088208 0.018387627665727275528 0.36457564552383509016 0.030706157865150351949 0.030449316592198964054 -0.013117998442033535281 -0.015451678351864507957 -0.96114420866193994719 0.0075903313311630367377 -1.5170960917412590341 -0.048182450444841463943 1.876438196222800725 0.4367062368102419545 0.29680863065115076216 0.3229944610049697018 -0.040841845753676102315 0.039139348606531662589 0.030722280364100903322 0.38750334256249169274 0.36645265042585573045 -0.65775210556856356092;-2.512320154697580854 -0.84900216861949096003 0.74902527840963695205 -2.5674241939799595791 -0.04454892856067725615 2.4370010802086596868 0.40821491517832603213 0.18090448605417733252 -3.6898158652366341848 -3.5096868692585645455 -0.64103477385459350923 0.22921466338406859542 0.0029985492158407841407 -5.0127102530430480698 1.9995244202766428998 0.27560820159504839655 -2.8721759273661136014 -3.0593881067715678057 -0.12048232046589837929 -0.50159719063271557271 2.9170335293302795954 4.6094822850257868296 -0.085904108280502197714 3.4378554971356249936 -0.54830659999334152577 -0.054973087018723179598 0.45985925133387051122 0.59799925191189273743 0.1565432761000903894 -0.2024746095155341985 0.44996421549671522522 -5.3214263072990792836 0.74313456977775871337 -0.43470732437129178116 -0.27897974978603268248 0.15286438693043000181 -7.5288252387127698739 -0.29977273648187718891 -16.411327213484785403 -0.32285113748844024295 -4.9487169921629252656 -6.6160601094051489923 5.1505526361491655862 7.4706012083563715365 -0.36257850596586349567 -6.5589043303250820216 0.91121935704938450407 -6.7074961332934712743 8.8256707669323333221 -8.9438455125234188614];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [0.540523484900437;0.788527964418997;2.39797429377933];
y1_step1.xoffset = [-1.82517310309568;0.576613863304083;-0.415768410298986];

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
