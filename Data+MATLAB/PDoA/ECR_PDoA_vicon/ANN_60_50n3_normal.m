function [Y,Xf,Af] = ANN_60_50n3_normal(X,~,~)
%ANN_60_50N3_NORMAL neural network simulation function.
%
% Auto-generated by MATLAB, 13-May-2020 22:42:16.
% 
% [Y] = ANN_60_50n3_normal(X,~,~) takes these arguments:
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
b1 = [5.8809631542639593249;10.917175090683441496;-7.0980631371562994403;3.8621709879677710298;1.9830618313092682747;4.4687293093360196394;1.3247549294588079061;20.104396058959430604;-5.2259686922431711764;-1.3742142833776169741;2.2756439351235480117;3.7435211543780475552;-11.142789017301444332;-2.8666331620287852644;0.96489817010503453609;1.9113700970483078478;0.13059232170500920112;-8.808461198713548157;0.27132456236101171188;5.2460608502071268688;0.85881930267130635048;0.42285340237305774069;0.87644322399086149744;-1.189543625458414855;0.084560894197449470999;0.67615307170463356812;0.6753642550363913033;-2.6768190183128379722;1.1133280248606911567;-1.8496138674277990965;-9.9287160824821683036;-9.904843519695154086;3.218246966891106009;-0.99707919295335922794;8.004763614184007281;3.2392157226234776779;4.1351260133576914768;-2.508293247793763836;-5.8793054523698478064;9.7332660683309732974;14.30350245398990161;11.36414288293943109;-1.3735818953712366497;12.47945975724863743;12.441313901665298047;13.809328128309537931;-2.2637321850167206883;8.0616355635198004848;-25.321742207589004181;2.3437323306113158061];
IW1_1 = [-0.55226673342846788195 2.6049813565445769648 9.1558574393482636822 -5.9200673828304744006;0.42312420795807897944 -7.8371167390397120656 2.0604305376269667072 -3.9425180466684004443;6.4944937755095972776 0.092769852527283452881 -7.1071374716571877883 0.50730590579111245475;-3.1994167899822767431 -0.74033695412688915738 0.26636960501396067524 -2.2663383052997247979;-2.4796590505989324349 0.097247062791868690845 -0.57623652704961192406 1.3348872907991606862;-3.9972511454693302468 -0.79740958687738372479 0.23349545517830641583 -2.3904748672479549931;-2.2592515874016401156 -2.0932283901210659849 0.43052050610962183663 -0.69539000717700116816;-25.417982558253886083 0.096361993964899667064 -0.93821909826336147287 -9.6952002969933026577;5.0756037478428570608 7.5544202963234736714 6.5029229460667448137 2.0391330596013168552;-0.41178472655857728535 0.25888699425970895796 -2.4707452662342799243 -0.22100557741857154714;-0.73876525911260726431 -2.8430909730796161128 -0.63918658927803806424 -1.7644070635860269469;-1.0945576029190962064 -5.0812198417754341762 -0.93371327509543866618 -2.975824467131491069;18.313535906661517316 5.8136399750942215903 19.376286908776968687 -1.1216742338487211139;9.0824766404326293667 -7.928733363105198606 -8.8820475088639039285 3.9374087818250735893;0.17030933825972344176 -0.36742496055444207048 0.22391696340456304637 -0.43373101511985306944;-1.2454111464384884922 -5.7245805422323314815 3.9253183670074371925 9.5488407703934310433;-4.0417290588855694367 -0.77103785734596752555 4.9232782533189292096 6.2290457409584609749;-2.3375275992396549185 -6.6140089679925724298 0.71573842743259641352 -6.144040724510428042;-9.0568648934016788843 3.0407717359175898686 -0.70386387768430647149 0.77304584726821012897;-0.25353139763275267526 7.7655391061741259051 -33.666238373019751862 -13.747833499229320964;-2.5177408745094602693 7.307399324863611767 -8.6799558631858921842 3.3216908501955795252;-0.033830920063837435774 1.1656314368723181829 0.070735620513091887474 1.9214494161610795597;-2.5571169570454093112 7.1527948910287877382 -8.2812099625415545034 3.009927473461082581;2.0396130047082290382 1.7920150215226757773 -0.41936284692342440383 0.86266945870380595807;-4.5723852734323013891 -0.68220959586546670028 5.6758127527742070484 7.0528306049553552981;6.8298762472962977199 9.5310817519435442335 3.1123131385354638745 3.7807081333242074095;-2.4641589789663753685 -0.38967473333699392812 -0.599565845527117669 0.26449940056828780133;-7.8049712118491765267 3.3720250102989313135 0.82756781369228249456 2.7776803530318532331;0.48174680259732910459 -0.19320682383576784358 2.3295117622451395611 -0.24608684143133574551;-6.9871340749254562397 -6.9370819547592450505 1.0690853759181522253 -5.7128044007470251131;-17.343576012541880971 -12.301905127595160749 -3.2163087366659115496 -26.394618093601408759;-17.501135815831080578 -12.375068755593121139 -3.2455956819329547791 -26.330265124348581907;4.0353560320238681669 -0.15307029486164486221 -0.20857270762190682767 -2.6797371468563397912;-1.0926217578730719193 -0.14313831750512853724 -0.66441122933956109353 -0.61397118305937303795;9.6309302157609248951 2.4747555463621795369 7.6747460024371854104 10.629537731346648144;9.4320265244057122089 -5.22115317542555335 -1.5154737502104653757 -2.2233290708629236399;4.1258482392263777783 -0.68071530779102962239 0.51354271407688201201 -0.53880694932621886473;-2.9946761467442022386 0.1465091459576446653 0.03415457684187236248 1.2624539425876850629;-7.838385708019333542 6.4479806743604868302 10.359665338282368907 -12.005105854145007527;12.861884464977341835 -2.6880717589094880182 -3.9351922195894930567 -1.9936545544191124346;15.742358523219973421 -11.966651853404053796 -1.2815741206565911536 8.8545035421866309377;14.953104148916342098 -4.1667137044237190224 -4.7992951840644355599 -1.2112340445489255814;-2.0187855124441815846 -0.16508994809988047914 -0.79145621052023096276 -0.51631482848413967623;13.772085317239323388 -8.8770322212723673516 -0.8935669285496203873 6.2530318562555713413;11.915106672362385964 3.0114270286856239522 -7.585698497058847245 -9.3348359834813710023;22.021085987428847375 2.3152124211762665773 0.38483032817608664455 10.462464110617245794;-0.7213240185220173295 1.7191165486365176385 3.44459802156378192 -0.15495098862229073067;5.4020182486148655698 -1.1309678051403186938 -8.5945655950059727246 0.90282965699945960658;-1.5875633518993548243 17.45544101873067433 -6.8540304066287758289 9.9926028971575782123;-0.073158745571878361358 -0.056723405309499187743 1.9489665224246548458 1.7775720582005478665];

% Layer 2
b2 = [-0.36915003487063469123;0.86822467029261396476;-1.1448471740642076622];
LW2_1 = [0.048604914141298478925 0.024496272629951319766 -0.0057409543956662657457 1.368730635435180254 0.0060940396688298776456 -1.2789462086983858224 -1.3409905824136569841 0.14103288627740784289 0.00012014402330908641181 -0.48918284201168121816 -0.058736705526050106418 0.024527194515096575195 0.0034017642045952580995 -0.0064227388352536508506 -0.15933648485869519873 -0.020184052195818903697 -0.32927783899233248333 0.037526222681725833474 0.043191117065966816435 0.004384757268864125819 0.11912495002516784726 0.044100283159279031475 -0.1079544440361526747 -1.5924458486773436228 0.30442656006779955646 -0.019166607646926295955 0.43760510257607643236 -0.23276439840477150534 -0.40495880213882406995 0.038541898460415689731 0.03042192771373306362 -0.0093513265292312908183 -0.04117914719183753125 -1.3511709328822834042 -0.030550071895144654405 -0.23312964423462298469 0.26097259998427935201 0.39383115743077062865 0.0066463794859928397238 0.67634021864377480426 -0.45334740358653163872 -0.63923107659495115307 0.8014005894793043705 0.54713842394004330583 0.074508737045443251823 -0.035486182710028180143 0.28897268456397101621 0.16015552523409692931 0.014877437375723556734 -0.32567641156417326265;-0.040950008329332446277 0.022768571277575581913 -0.042450252251491578381 2.6998321478643809712 0.23895918274788197078 -2.3444728663234428723 -0.63301365764771810873 0.1686242224189499761 -0.018179999392854053614 0.35373583445791711988 -0.42774565514802498356 0.21031361990428720787 0.011422374821112690757 0.0018711891762804770765 -3.3048524574709552404 0.021366799984275618163 0.043870086606426050846 -0.12356497389366186135 -0.013931271354443112293 0.014323615271815462111 -0.12292241005434630108 0.090811336898609942847 0.13658748308435997298 -0.8361719985297364488 -0.033209129467246134571 0.006567003480384118845 -0.12123990163528031161 -0.15917811833603964788 0.50131872075785643972 -0.026488094448355915117 0.23161032496003786108 -0.24179385088774679735 0.36284355693338554705 0.092933045927070245806 -0.0010292978617897649649 -0.068920838602529788108 0.29855233258004998387 -0.13379125076804720962 0.003682743333990323286 -0.78386413876118876676 0.99219318717744742209 0.77353420337010725394 -0.097322892521935419619 -1.2614575689753360077 -0.0099393202493015791327 0.061730508776325888898 -0.059730529029382861683 -0.014197220673421015111 -0.055699092731636277998 0.080834010926582613998;0.34750854957418414459 1.6756317160370419739 -0.15179572124807086397 -1.6693532820780976333 0.64695942324307154081 0.57517985477461741617 3.6769932974060921538 -0.010396283591020773207 0.18030729456637828445 -5.9121737405848326929 3.8879500783710732037 -1.8887448512499931397 -0.10289643385357941097 -0.11729303516018964315 -1.9186081407824160383 -0.16265258863361553576 -1.8114122498197295918 0.15184861141859801181 0.26695899668782158143 0.13674461442321836357 2.9171516330051683319 -1.1999096741949542011 -2.9077761497621357201 4.6839624887191826375 1.6151839385765001111 -0.42768598189728324055 -0.022230167126761068985 -2.3502214641889969116 -5.8416824410281593671 -0.45791680182689337153 -5.2197823479463290042 5.361725618817564154 2.915002779961258117 -5.2133974776623590586 -0.36297989462393170923 -1.9271296413006990811 6.957833223610998985 7.1343367064876410666 -0.19564692885389897703 3.8761263091072732934 6.8945948925523579121 -3.1715937559304001603 3.0448191836498232909 -9.2059833331763201159 0.81143565656005312015 0.35482962172133308831 1.8475585045394791184 0.63347580813791437571 0.9843191813921272626 -2.1708965418644399215];

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