function [Y,Xf,Af] = ANN_horz_500n1_normal_compiler(X,~,~)
%ANN_HORZ_500N1_NORMAL_COMPILER neural network simulation function.
%
% Auto-generated by MATLAB, 27-May-2020 20:11:12.
% 
% [Y] = ANN_horz_500n1_normal_compiler(X,~,~) takes these arguments:
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
x1_step1.xoffset = -166881;
x1_step1.gain = 6.4553820133691e-06;
x1_step1.ymin = -1;

% Layer 1
b1 = [699.8662951874217697;-697.19663706016922333;-694.38878079104563312;691.58316630770968914;688.77755511024190582;-685.97194388847412938;683.16633249972494468;-680.36070065467515633;-677.56203243023287541;-674.75049560273464522;-671.93488583403632219;-669.12058157008470971;666.32356908327960809;-663.61667776166075328;-660.76991299258202162;-657.69355616200152781;655.23224215748723509;-652.27822452385203178;649.21249559538682661;646.62711588303034205;-643.94554758154174579;641.1343449591352055;638.33113179133226822;-635.625943100252357;631.4955229821582634;-629.19542962248635831;627.18729304606245023;623.99271123884011558;622.49295899672813448;-617.89501368511719193;-615.2759773176909448;612.70211419283066334;609.71941144287438874;607.51075801974241131;-604.75496419827925365;601.49822309286309974;599.85597507377622151;596.51802894255013143;-593.96085626548904202;590.60523132047762829;587.16071043997055767;-584.82190959490469595;582.67076934770784646;-579.60862035873719833;-575.8540699234853264;-574.51653966135768314;569.97322182879440788;-568.52375188712755971;-565.31520099764009046;561.7653267988183643;558.55124906066157564;-557.11263527145854368;554.41126670371886576;551.03545723951447144;549.33110369418670871;-545.12994261546350572;-543.07453017632542469;-540.90774087565660011;-537.32712745698211165;534.50149263172397696;-532.30478369492414004;-528.83581872100035071;525.92913948190926021;-523.46466354887706984;-522.19541087634286214;517.70633650069078158;-514.51685465740376912;-511.78477845891700326;-508.52884220128214565;-507.0521407760120951;504.15663469968052368;501.32594114757631587;498.11963452780793205;495.21501683311373654;-493.16270658798146087;489.17155228413651002;-486.42126313076619226;-484.49903539033090283;480.72362467592620305;-478.87211942316616842;474.99238673117315557;473.09859155499754024;470.44116726811478202;466.87555541579251894;463.58996941466250519;-462.50018130130007421;458.49766982700077733;454.76419720743484731;-452.7399237880241003;449.99879889541892908;446.48844786577649302;-444.27848225994966924;-441.84254856971210756;438.52296961708759682;435.90094506566390464;-433.40410565008266985;430.86547704058534691;427.77014154534828094;-424.64502547003633026;422.30939441755930375;419.42049419262991705;-416.47075723490360133;414.08855781798217777;-411.83333377662341945;408.41273795174635097;-404.96258968313367177;-402.44224178825726312;399.55983100249505924;396.75508378390435382;-395.10757299408055587;-391.65497137756517532;-389.25466713459775292;385.7670655689669843;382.98831955217616496;380.14287814469156501;377.57156701038115898;-374.59113879760900545;371.8573386676604855;-368.85042049392882291;-366.12507920703990294;-363.31342474281967725;-360.71755487438241516;-357.64665068912421475;354.83206706458832969;352.11938434023596756;-349.35423679636977567;346.49119876170277621;-343.62296108565504937;-341.06067964038902574;338.06586084343155107;-335.06051585239123369;-332.57436471340753315;329.85950075343998833;-326.97697010358160696;323.95722085534862345;-321.45494158781411898;-318.47260162062474365;315.31844590580698195;313.38796946705758728;310.76837609170547694;307.16519858064089021;-304.37539520150517092;301.59474756546103436;-298.50904041695395108;-295.93482472008827244;-293.12819495686989058;-290.23093177805509413;287.54561766276202661;-284.80866902953380304;282.3609756662654604;279.3133705047021067;-276.1431182964441291;-273.25222570757313179;270.65852004073428816;267.78533665242684947;264.96564991868433481;262.61325405480562267;-259.55469527650421924;256.71909127757629676;-253.9325227095842763;251.21539255821232928;-248.18743435641582096;-245.70231751274397425;242.45718455123437707;-240.01357516594546837;236.97187725024849669;-233.6150962187124378;231.67051755077724806;-229.19964309025414195;225.97777546989118491;-223.16939213713058621;-220.23624953813143179;-217.44418099420531121;214.49624667660873456;211.44957699799653028;-208.89553311626130494;205.61163883155035137;203.45441007829697355;200.65971816879326184;197.81310447894585991;-195.09144721376881648;-192.19901644939460539;188.93733946968922055;-186.62224067219091239;183.73689609390555688;180.93511344372893745;-177.80419335125483826;175.49863466367614251;-172.48692673421260224;169.41765526856485735;167.07563904072893024;-164.07727226533430098;161.44249454095825058;158.46236631673176021;155.81713126530081581;152.91612121482589259;150.02204557319078049;147.12563231565351884;-144.46034348038975281;-141.66163177511361937;138.83762121670147849;135.93380456757876118;133.32714492455897926;-130.39975115114299342;127.64682596717570107;124.85132778021205979;122.05123231743721135;118.89194343941771592;-115.31584739908134907;-112.99675695680456045;113.09630617740651815;-107.57949456233717456;-104.51436794779064599;100.7434862039793444;-100.11890526853591155;-96.858976997456466052;-93.078288225369419706;90.646141035360017213;88.314908657111885759;85.310581990036439493;-82.709496956365427422;79.970608951035003997;77.003673020641826952;74.401844761090814018;-71.498254479777543224;-68.698796046593827214;-65.958461084293091403;63.174533165309547655;60.356171681217361424;-57.408509033908728725;-54.755216716680394029;-51.881230266675707696;-49.250252796373359843;46.346281706096405628;-43.51840495524363206;40.635483870236690507;-37.912244068751050463;35.067057773823066213;32.278671992081228836;29.430448370234589817;-26.652787691759645128;23.687961642467950441;21.198989226968869559;-18.156204699135805924;-15.477915491874830067;12.639776176790284978;9.8214759295629079361;6.9704597850427401085;-4.178259702294254474;-1.3802890358563024709;-1.3920612548617123938;-4.2004972230931647559;-7.0213750116834274806;9.7519305238963109872;-12.636027415723761536;15.412052359099529752;-18.291538056532065326;21.12193097898834182;-23.700349962376304092;26.794797398721538428;-29.383989228809863192;32.354193789059820574;-35.025927498790245807;-37.915090570813568149;-40.490273568631025114;-43.673123336921662485;-46.19374798003491378;-49.490792368961024295;51.616793112977710223;-55.6211830119284798;57.502434497420310322;60.416996497373034458;63.115121475204063017;-65.931890509537979028;68.737657327429857901;71.519031800916096131;-74.319651923079348421;78.17702183186672471;79.991647347041450189;-82.766250138946389825;-85.58120896749673534;88.243038440990261506;91.050884225589911125;-94.000662937461612501;-96.953262601974131485;99.753053515331814083;102.4232734662480766;105.39295970595921403;107.73289307433790896;110.7655389075716954;-114.03734483657900967;-116.4026349969927594;119.36182470920333287;-122.22344071694261913;-124.88463917794355496;127.68610168228130419;-130.49475062287800142;133.20279019630254425;-135.97838760698593319;138.86330897736189627;141.81981108906475697;-144.38032405923772217;-147.40080222072410265;150.16470633607372065;-152.79554261983895458;155.85405923959808661;-158.48927206456522754;161.27828811146767407;-163.92011369237545182;-166.75908590323842873;-169.94669487759472304;172.66068042248403458;-175.47881157119118711;-178.11143656195793028;-180.63038831409252793;184.14264076491332389;186.61676249179561182;-190.00109489009014396;192.04239146587374876;194.97236010195092604;-198.07250192933389599;201.34423281306149534;203.74828789591057898;-206.19823071724991337;-209.00650461635555644;211.84830118753598072;-214.95380833455223524;-217.90291787819231217;-220.15698213990506815;-223.67102970136977547;226.03109085549445467;-228.72678245973352773;-231.44655923208799209;234.46752134699408998;-236.74050998379104271;240.17518854165979292;-242.6784499132033659;245.04383581115152424;-248.01934566901653056;-250.75858163412675594;253.99882328342638971;-257.26685553393332384;259.60475967630770811;261.33191279564499609;264.30659220134003817;-267.72460013502194442;270.30620994561246562;272.38117223238288034;-276.69118520791744231;-279.22136240370809901;280.22589309087157972;285.77850094950952098;-286.6451506402898417;-290.67960941204006531;292.77492485394503774;295.69936572416816034;298.62453073247866087;302.12333449133836893;304.07947020984028086;-307.79332325264942938;-309.62804758931554261;-313.03667759491361267;-314.4220780738664871;316.97150453098532807;-321.17970151852335903;324.09396088902661859;-327.10221045885560898;329.62052851587418445;-332.36815053849011292;-335.3088625604929689;338.18357188092778642;340.36521569689585931;344.14298081575122978;-345.72451070814287277;-348.95055256704006297;-351.77765317837497605;-355.82387497448485192;357.87959058148618396;360.179997165875875;363.27266357631356186;-366.16520485600563006;368.44981029379130177;-371.74906542991908509;374.58442689575730356;377.37624569123278206;380.23171281035325819;-383.35311319146904907;385.65305182922151062;388.67258834354072405;391.26609267912152745;394.16568678381872814;-397.44254832275186118;399.91827097818594439;402.52638392672326972;-404.62479800442179112;-408.45173935489611949;-410.74102596784069874;-414.93451228904552863;417.84063538063196575;-419.49812847474146338;-422.29791576788352359;-426.21248486907046527;-427.5562343463579964;-430.58751536845147712;433.7943011576451795;435.65774858499401034;-439.35850198927448673;442.39820546329389117;445.85254972955601716;447.84118150449489804;450.03154409698976224;-452.5070753911897441;-456.01011326424071513;458.31661713570258598;461.66335476243187941;464.21869063669885236;466.88685355256404819;469.932665973586154;-472.74314025427167962;476.36205409170219127;-478.89466657994603338;481.16816633200556907;484.40272563545607909;-486.85555554403634915;-489.63943931562135958;-492.487417911503087;494.39014786025654757;-498.68171760501672907;500.64984064125127361;503.64343180752638318;506.23430720821011164;509.29710780748774823;-512.01060994563874829;-515.1978175380365883;517.82043575794295975;-520.43244648050858814;523.22024447798764868;-526.14847288445832874;-528.43964171846573663;531.59003834232794361;-534.51117182197731381;-537.82821565751157777;-540.28007414742944547;543.13025882842464398;545.79953133944161436;548.45462775582666382;-551.58503573311645596;-554.02934602495395211;556.44059156427579182;-560.09584865354429439;-562.12336633253141827;-564.81854602944258659;-567.64825646754115951;570.79144699527739704;-573.80953919913724803;577.55090233314012949;579.26847174031217946;-582.48807171715475306;-584.06169750329831913;587.47341174924224561;590.62871703264977441;592.93608245906511911;-596.54112169060306314;-598.94671016320228318;-602.75563994876483775;604.9160774037951569;-607.48555656415862813;610.2804399276096774;612.91368917129364036;-615.80992994007579;618.62110550672491627;-621.53260612521751227;624.4892911457422997;-626.84175174519987195;629.93990603509325865;632.66215112543670784;635.44292674799203269;638.36015118314526262;641.09096941664972746;-643.9714605272965855;-646.66873895269657169;-649.41053321892070471;652.30033376165158643;655.13177748462362615;658.11490318993105575;-660.88964939644017704;663.52659572558502532;-666.33269629916128451;669.12055112568953064;671.8815650607665475;674.74947239583900682;-677.57283108366880242;680.48607844535160893;-683.16883273741905214;-685.97195828511371474;-688.77755503029584361;691.58316633277661367;-694.38877755511020951;-697.19438877755510475;-700];
IW1_1 = [-700.13370481246477084;699.99775171738917834;699.99999676406457638;-700.00000002495562512;-699.99999999997851319;699.99999999932572337;-700.00000015986438484;700.00002006761246776;699.99331775440407455;699.99903241125446129;700.00865671461349393;700.01694853009450981;-700.00862525465856834;699.91501221830219492;699.95421260123748652;700.20859636284103544;-699.8854378888094061;700.02458855706981922;-700.26511801262620338;-700.06110120629398352;699.94675775106600213;-699.95200070389080338;-699.95004281918420475;699.85965362529645972;-701.05718926574252237;700.59851160970185902;-699.88094989596868345;-700.22990743707987349;-699.06806309135504307;700.65591108723765501;700.48872753926423229;-700.28283288849024757;-700.43691172092769648;-699.91708490108828755;699.87391130644107307;-700.2632261643517495;-699.26491100392195222;-699.7225749851143064;699.51290551460249389;-699.97731833174054827;-700.51635012688370807;700.12279005642130869;-699.57832293348837993;699.79397499145909478;700.57545234374538268;699.36723693375506627;-700.79313082754549669;699.68558267181697374;700.01204791220777679;-700.61026856265141305;-700.93402167228873623;699.84119138833079887;-699.76478749165721638;-700.20987185371018313;-699.34622402055390467;700.43835283320981944;699.85366604398438994;699.36085437791211916;699.95950318155428249;-699.97480865278566853;699.51451502726354192;700.01659834349311495;-700.09230873006868023;699.83841016068561203;698.69374349707447891;-699.94744026127580128;700.23012380509806007;700.17480111381007646;700.50128899768776591;699.53722555521494542;-699.60474823362426378;-699.62482150411449311;-699.91178059241030951;-699.9825320502758359;699.45316664272047547;-700.28431921358890122;700.24472060818061436;699.63276377350496205;-700.30017341945870157;699.64745699100183174;-700.379934216865081;-699.76082494521176613;-699.66326924579948354;-700.17219511514474561;-700.49060963914007516;699.35616629789842591;-700.1421797769812656;-700.74658944517295822;700.23665957582818464;-700.19383735710357541;-700.64221170171117592;700.26210530187734093;700.02477790243199252;-700.34743116223512516;-700.23189529177602708;700.04053467959909085;-699.87319825365625547;-700.05212885390187694;700.24591006825107797;-699.96060125593635348;-700.01097384103456989;700.09648842135538871;-699.84578428545933093;699.52358568958459273;-699.88520280265368001;700.25920436811043146;700.09323249496935659;-700.1368718618590492;-700.13550726653863876;699.4822288673700541;699.8478409796066444;699.62366485632492186;-700.00229713223518502;-699.98750012379798591;-700.0096744542969418;-699.88284473060843993;699.97740956616598851;-699.93943779836513386;700.04608456760377067;700.00372943234719969;700.00664864558132194;699.89863768277746203;700.03503340442091485;-700.03940372814190596;-699.99224208111002099;699.9721835991745138;-700.00086106370781636;700.03155899441719612;699.9127141066655895;-700.00490559423155901;700.10040964721270029;699.94765197432843706;-699.90575730891862349;699.94239687876881817;-700.0420455433859388;699.90227522895952461;699.98369968605084068;-700.14103565207358315;-699.74856384264649023;-699.66780588378560424;-700.02145498395623235;700.01454058652075219;-700.0036651996490491;700.12314449810719452;700.02412716700439432;700.02450835658760298;700.06203230773110135;-700.01171007135792479;699.98367414716437906;-699.83976203703673491;-699.93762102777509426;700.08239215152923407;700.11541438607389409;-700.03174637488257304;-700.05727969661052157;-700.06245706091567627;-699.89156253134808594;699.98682258706560333;-699.9979931949363845;699.99121914525335342;-699.95931756185734685;700.03851777844772641;699.92592005371636787;-700.07875595217660702;699.95369739790294261;-700.03470797952229532;700.21818895242233793;-699.93301260545854348;699.82245633307650223;-699.95940374049564525;699.96078573274292012;700.00089013858394082;699.99700510136426601;-700.04016230603770055;-700.11306765535300656;700.03669020160191394;-700.1762975092610759;-699.98610454804452274;-699.98334958553198248;-699.99451134607636504;699.97161351471663693;699.99608116105775935;-700.11944190523058751;699.98696993614589701;-700.00803298358380289;-700.00648215426951992;700.08917562919191369;-699.96312916951455918;700.01427443472312007;-700.0779116927932364;-699.96603586141611686;700.01154887486006828;-699.97200238967343466;-700.01235197979724489;-699.97670675944596042;-699.99776641582650427;-700.01669313057334421;-700.03565929614649122;700.00585635670540796;700.00431090641143328;-700.00781590217195571;-700.02694080479091099;-699.98836717450512879;700.01123751106513282;-700.00145195705908918;-699.99969911532423339;-699.99871646575832074;-700.0588048086380013;700.18481930463758545;700.10401072714807924;-699.63769079738381151;700.06974455222052711;700.10195538681466587;-700.24400151298345918;699.92821792132724568;699.99087418805038396;700.12160421823443812;-700.07057681763251367;-700.00772608095576288;-700.03142714813748171;700.00710181082433792;-699.99877253829174606;-700.01650612725450173;-699.99431248165376473;700.00447395374715143;700.00384546764405513;699.99755001380947306;-699.99561689804977505;-699.99690046722207626;700.00857614544975149;699.99640366768176136;700.0012290076113004;699.9892852357272659;-699.99643465015867605;699.9980106137961684;-700.00258097567052573;699.99801569736882811;-700.00010153738287499;-699.99938136090372609;-700.00105944875974728;700.00001201214047342;-700.00487185041595239;-699.99483574649320872;700.00209348935027265;699.99899810675253775;-699.99968333858930691;-699.99999849072764846;-700.00036053955420812;700.00019925630829221;700.00001676641079484;-700.00001316165855769;-700.00000096093640423;-699.99992963019928993;700.00095399940437346;-699.99979237391596598;700.00040203838284469;-699.99851597928193314;699.9972960703228182;-700.00482750246328578;699.99450190265531546;-700.00308384462698541;699.99582936911167508;-700.00214822998430009;-699.99783396509644717;-700.01091938853926422;-699.98826088740861451;-700.00614806721364403;-699.97271129017713065;700.02152171645366252;-699.92828189562794705;700.0009881124333333;699.99154449291688707;700.00097669125079847;-699.99999763205619274;699.9999813248581404;700.00245497258140404;-700.00311696807420958;699.88953041249521903;699.99644695577728726;-699.99990849502262336;-699.99871962423605964;700.01677653921888123;700.01715290321351404;-699.99799598392496591;-699.97793217418336553;699.97819723504744616;699.99730904049590663;699.97229492235726411;700.04331051981944256;700.008920979424488;-699.93354844207135557;-700.00510934733904378;699.97879853984306919;-699.96871836050320326;-699.99381284772277922;699.99437236902781478;-699.99367301114500606;700.01206358515105421;-700.0181794652029339;700.00284272501710348;699.97224635656073133;-700.02222965753117023;-699.97758968592290785;699.98612983781958974;-700.02405551744107015;699.96806120998917322;-700.00619460793302551;700.01023691454076925;-700.0484375245252977;-700.04154892594181092;-699.94965330608931708;699.97143112813682819;-699.9680198452484774;-700.01137382846138735;-700.08599265717134585;699.90190504296333529;699.98822263623856088;-699.83140774500134285;700.03870445034840486;700.00535821888217924;-699.92137539657994694;699.78703805779741742;699.90079125042768737;-700.00445635042513004;-700.00342686635326572;699.99230236280288864;-699.90053010694032309;-699.85479829116320616;-700.02614583348110955;-699.8003272666887824;699.94194067533874204;-699.97729027434263571;-700.00541158932469443;699.9332759335859464;-700.11198579923757279;699.89925135513988153;-700.00245489179405922;700.15619767732493983;-700.09830753618234667;-700.12444021516432713;699.96605287512852556;-699.79707128763811852;699.96844288011698154;700.3706638254899417;700.31285165229746781;-700.08026506635906117;700.16795196227747056;700.45529714594078996;-699.86495188037326898;-699.97542183058180854;700.69799454731014521;699.58980291290129117;-700.38196694642897455;-699.87617061241758165;700.17279797596938806;700.12364371086869141;700.0739553861604918;699.776312503663803;700.14248155587199562;-699.74524166465687358;-700.17395064927950443;-699.9053459081200117;-700.54507696964788011;700.66545357797679117;-700.02841178485812179;699.97805388610788668;-699.88394653309433124;700.01803301185213968;-700.04592817208128963;-699.98157646033132551;699.94785599763372375;700.25142606600559247;699.77692556898614384;-700.3799750741607113;-700.17439464561311979;-700.16413422590369464;-699.53575328585793613;699.91600375451650962;700.17557603954992373;700.02770234250613157;-699.98247438021019207;700.2571281149079141;-699.99685948585556616;699.98114522042772023;699.98840841257890588;699.96086075978655572;-699.78776020728048479;700.06559409516569303;699.94734174876941779;700.06465965851054989;700.01408354512091137;-699.74568229822341436;699.93225765064528332;700.04484366719657373;-700.45596059010313184;-699.86253190488821474;-700.16491006150249632;-699.34383006324139842;699.28107054983138369;-699.96336035709566659;-699.96758819408591989;-699.29302103884197095;-700.18456308770032592;-700.04610140197621604;699.79753365498936546;700.38296714517275632;-699.82222055114527848;699.67497082067052361;699.26198992202205318;699.77846537217305922;700.17327511378823601;-700.38783654248493349;-699.93689095155593805;700.26219601623483868;699.90753290743248272;700.07290846233013326;700.16529778446431465;700.00489540998137272;-700.00158732036391029;699.4493780377300709;-699.63190343309645414;699.99576856075987052;699.69960911263558501;-699.94263318404841812;-699.95726820988579675;-699.92833794393504832;700.56589845013468221;-699.51189471912539375;700.10875122975903651;699.97357123449876326;700.12894485870481276;699.94265903118673577;-700.00982146137300788;-699.7287806772209251;699.8629140296333162;-700.00620242916579627;700.0195467635822979;-699.92763461092829402;-700.31519209468353893;700.05559226311447674;-699.96794380909409483;-699.57468391177974354;-699.84662367593307408;699.81001391376037191;699.91546299317087687;700.03452717323750676;-699.77842282806739149;-700.06238407909768284;700.37727384678612452;-699.69934733576235431;-700.32421725546532798;-700.41413364945412923;-700.39565836136057442;700.12212224264453653;-699.95049740382478376;699.17742438480559031;700.07524222252800428;-699.73040606548829601;-700.75900725723602136;700.25311617775332707;699.95929340121119822;700.38151293649707441;-699.70284225613943363;-700.04368050002528889;-699.18090392011606582;699.73478898464577469;-699.93879174764254003;699.94736877890557025;700.09790839144579877;-700.01887300951239013;700.01417294824716464;-699.92016267064332169;699.78526547069668595;-700.18997786948443718;699.92783640973993897;700.00284443041152826;700.02543220986501638;699.92369470668666054;699.99180152616713713;-699.92266098536299523;-700.02246157120589487;-700.08199428717000501;700.00397642311884283;699.97979204424018462;699.81281093493305434;-699.84150744994212801;700.00043190890403366;-699.99997035205967677;700.01698864846161996;700.05973220106034205;700.00002504086114641;-699.98281229987833285;699.87810070448222177;-699.99756957914416944;-699.99998600376204649;-700.00000007769824606;699.99999999989165644;-700;-700;-700];

% Layer 2
b2 = -0.36610374879199819587;
LW2_1 = [0.57637915723786548039 0.63085277488177571303 0.18990740442388634457 0.69006985994025160647 -0.045417626507574408723 0.88952739646819078967 0.77826348025644254136 -0.35913219351169300975 0.43759115847769025631 0.0064913825660175717563 -0.044063685069223722246 0.7307799015817344479 0.70166768254353861778 0.0084763323619624023758 -6.575900157676202798e-05 1.1175748822566224441e-05 4.8652521122934241568e-05 0.0012449606584295589556 -0.062178622297776875438 0.077530285446274493921 0.014376144903201923397 -0.0092436329148354885038 0.011254160104689740415 0.022346678553073373585 -0.056065385876349141392 -0.097190231699937137955 -0.035935969433053494004 -0.049244840683215815313 0.042632971594516809455 -0.016118442969899206274 0.03103340954524881623 0.030369456919926318683 -0.013690662657751678788 0.015675950033506289016 0.020097854576651905112 -0.026486205950767199857 0.01081497868666765616 0.0087255124720792571341 0.015931730963162731712 0.0066657023609988080154 -0.022204864727670418217 -0.0096882207856855660777 0.0038377604673183228837 -0.0081891561764841393212 0.04969288891249851875 -0.034297950714534848615 -0.024216041033752674916 -0.016928805400743221488 0.0097319195781773761011 -0.0056873095197010571297 0.018520240242283141779 0.030934540183685844267 0.019282150462026557275 0.0074375844475476383943 -0.022443461781215163359 0.0090919364097250786327 -0.0091228486162457343711 0.0096029010104368428546 0.00016205134619382932475 0.0056050749205033018746 0.012037619049612164474 0.0031035931165447781452 -0.003326020498369134 -0.014073211614170010389 0.010373075931193655869 -0.0028254138655378390269 0.0081645761353675572192 -0.00017174842061928478681 0.0038306130390454141109 0.0039478135055818330129 0.0045243656146647737507 -0.0059211499352032851556 -0.00092894339247666293108 -3.1787829805956721585e-06 0.017076830740866636577 0.0096621620551521035514 0.0095878812402308537938 -0.0064150425366093499768 -0.0076104922454668533921 0.0028274520128974715491 -0.010141692543722942432 0.01221214103452041011 -0.0040093564060080782302 -0.0074291784375997765508 0.024940250662202866072 0.030167627681750866508 0.0068181233457836158685 -0.008588473737803687183 -0.0061834522396771954972 -0.0065591574068107841994 -0.013876120547060171936 -0.017928306725018709755 0.015910906514639201748 0.0093588212913129180393 -0.017039564817701888938 -0.022905669075142812141 -0.017704566652117021547 0.0023009116389487117502 0.0063783887864220225813 -0.0032917953474041858515 0.0093576204104110945192 0.0034744722256762404326 0.004863277597090136066 0.0094719413169044655576 0.0047892482567669135765 0.011582420558123918131 0.0017229034507266584916 0.0029521820067182655105 0.0033453672451420925216 0.024256802819448874259 -0.0085336001730278478183 0.0081840675094765162612 -0.0019783693308286940879 -0.0049548212440886675589 0.0055297027249545790614 -0.005883447972877974344 0.0011678473806342728677 -0.0032591259551147913794 0.0007381592584804946627 -0.00057469812335722602389 0.0019686680446420017077 -0.0017180619315160044168 0.0011881150609989842969 -0.001324748249421392552 0.0012043672396505589704 0.00094719424818781974059 -0.00059864674768069523012 0.00093886529177626524851 -0.0015213069907413043647 -0.0020624868195524637474 0.00080560184729402929075 0.00030704874368941547508 -0.004179414937004809899 -0.0025985009895558614131 -0.0080053022915295025258 -0.0042801047673635215368 0.0020649010236660292686 -0.0079091138128582049588 0.009920038813921401577 -0.015012622902499821176 -0.0002172355736759826704 0.00060159967664249927154 -0.0016897068671469293692 0.0064582998801321562024 -0.001283773194439815149 0.0040013313287821658873 4.9138027996789514131e-05 0.0020757129634402489773 -0.003983219083453544071 -0.008324786717240297354 0.0026641949254549877543 0.0059594484775760906553 -0.0065958774840950747803 -0.0077717748092488887768 -0.008079870978323976799 0.0053225124536335670714 -0.0016167845744776954652 0.00089802511238416501179 -0.0028171055519176052377 0.0090407951540976781019 0.001594577164315724889 -0.00051735519605192517838 0.0052473853971787857639 -0.00077522037357230359246 0.0012315497041611171876 -0.0016979410391659054483 -0.010508959906530666514 -0.023438330802460966007 -0.0066020030184848416469 -0.005771269740622205803 -0.0055950047460034410887 0.010140747220448038599 0.0020869152586529297541 -0.0054263090186390276412 0.0057485418726477242879 0.0035655079482429801081 -0.0063645141672092030161 0.0096958711093482977811 -0.0081512661778661774092 -0.0035277168301261368523 0.0012924643159609543446 -0.010115833008486049827 -0.012692459359305697728 -0.0041161605195644797653 -0.0089312976952732828911 0.013128660797686830114 0.012224408356247976407 0.0065182869787843364384 -0.0029533380050302741708 -0.0035540321497651112745 -0.0044844928050811368181 0.0017314629896457657456 0.0036387486338306440289 0.0023270106520089819772 -0.0068412208309800784378 0.00090642769751555617858 0.0010746693039164066825 -0.0031403033024196632013 -0.004115908377877666377 0.0078766838490833199216 0.0042663837710768233794 -0.0050368471964276437941 0.013070979189187441685 0.0076109336101437118777 -0.0011770820691269080586 -0.0018263257314520308424 -0.0084391135248871942953 0.022532179083366445094 0.063537633235085946848 -0.08533858542157146676 -0.066472720308263136269 -0.035645359667451512187 0.047412269172344506662 0.11594049786447899186 0.17304944391454660235 -0.011744979503752065245 -0.0349434015176801685 -0.039145480016884004471 0.0016967121065238786534 0.015239242431678639081 0.016007951475339607383 0.00084084549189423021288 0.0043934491383849239715 -0.0049677355175276093416 -0.0026747376104770213129 0.0011756396243900102058 0.0023645338110221112739 0.0028468732757240254073 8.8888783622364866423e-05 0.0022663713431036441176 -0.0026142537741996904437 0.0019616621451576934584 -0.0022721872197696804836 0.00035166594023325635282 0.00082146742131187491451 -0.0005269649564545502091 0.00016556806425709541053 0.00044093910775602648778 0.00038410593692045483022 -0.0010028779685589939673 0.00037676473347390951113 0.001537213060750263708 -0.00062623813040546701374 0.00082181417442081375942 7.0150682550183999608e-05 -0.001075283938880132505 0.0022469207674965735802 -0.0023616360478188038949 -0.00083575830463284176905 0.0012847498183756918266 -0.00049593328627871194163 0.00081671258774604631354 0.0015940268456432929271 0.0023328833312950699418 5.9758370297083949888e-05 0.00034442500643611133814 0.00061617085745018765255 0.0025704788378789157274 0.0022081558583614364563 -0.0027064135081399710338 -0.0063222981918192953238 -0.0050320426165560549342 0.00079953935137200944031 -0.0035108044687487678474 -0.0011591380851242757792 0.0047578952055744531263 -0.0054224876498037265921 0.038521866893979739288 0.023732079193951594043 -0.077052697820663790251 0.11505353808449522712 -0.20683678754849416093 0.24503249948335134589 0.13110148365664686665 0.060384510574049199416 -0.1008207399252380454 0.068856217294029686826 0.64413731461402035094 -0.79072959010296095528 -0.20873857153134028897 -0.048322124969331116395 0.03563085249727372672 -0.051622800150285988452 -0.013658166453276837302 0.0069272033430817633523 0.015994968100968360442 -0.017092164765842920299 0.0062569529707497744883 -0.0021824854424799893481 0.0021110502967345789148 -0.0083571576832452976191 0.013629607336114079458 0.0078372467978482154155 0.0069830239472274246298 -0.0030862891723704905834 0.0020506956698131493383 0.001992305793975816721 -0.0021167673885243210898 -0.0047161475374035977115 -0.00030598189729974859235 0.00087280200720725280294 0.0012169929599483717345 -0.0011141370804429299481 0.0004954775156888934157 -0.00086789370058946958223 0.0026933512983508219908 0.0012475262537553198118 0.0047146625311074918593 0.0052280824809347523074 0.0043911103138483285543 -0.0071154015765800401536 -0.0036119276047960110348 -0.0064019746008819600203 -0.0045036981530783681132 0.0028823112735955089096 -0.0056997756948153170675 -0.0038396794165741353518 -0.029296509777077203024 -0.0048597151183733368687 -0.0069432767039561008385 0.00073050770337329757123 0.0089282414138858925273 -0.0091816958711205542976 -0.0058324558589101221234 0.00063268478432680673798 0.0017745280284503157139 0.0035652776759657108657 -0.01228560180959001151 0.006088164682085574729 -0.0083819249728854967885 -0.0020700613764007410808 -0.0021536540654171895323 -0.0005224220998561332702 0.00050367098565453922304 -0.010138720833364275528 -0.0073854498028769702961 0.0027821434785553480692 0.014103221040441985684 0.010589897635295376227 -0.013361706889531194467 -0.0017281629028572235464 -0.010296110900520363818 -0.013081277029266726725 0.022937181111494554103 -0.008488145077869099081 0.0013619491152303897855 0.027462119362494704505 -0.022149376236369210796 -0.02590225891142082168 0.019373656766563364384 0.021708863616774091987 -0.021291697335993002571 -0.029276381926757574087 0.020972349495439291528 0.011726194059074065631 -0.0016490049366034021782 0.0015818605140161755805 0.006849841542511359363 -0.01397821813232194739 -0.018267859934464055621 0.0081465699958782437917 0.0020140865045079540778 -0.009139817883203176993 -0.0087170480752469317598 0.00022307283077036639454 0.012160592183084337203 0.0043382064131216660496 0.0060632020853353032347 0.0005449882959754677068 -0.00059527245020875015156 0.0044647383110572739021 -0.0020958251936731846395 -0.011871555046383350934 -0.020374825015179828802 0.0052916398378748882858 -0.0032415672488689505071 -0.0098743705290090053733 -0.0034217793237060339928 -0.0033639105327013477803 0.002163038483673885605 -0.00043186327916556457515 0.008020731702150729614 -0.00025116787225257057592 -0.00092640739215829235285 5.6097012072333707951e-05 0.017091324476751518224 0.0082492853881519223602 0.021216219503627627152 -0.011156131425414966829 -0.012445130327114518193 0.020125722572791441417 -0.0063075244988385426384 -0.0064830817094151142396 0.017177791316429071727 0.014387490795911362165 -0.0068905181385429409985 -0.0085381256397366687289 0.010190089857297513626 0.023403372521407617146 0.023772417982869838149 -0.0098887266961262994258 -0.020240759826266586929 0.025965763562653187274 -0.016885660091533079979 0.012463992741914248913 -0.0090012120892092300356 -0.013057703964054447748 -0.0026327969525838036853 0.029087443504344615702 -0.03322884998367185011 0.01640514353166194067 0.012282625866690002892 -0.041816541282858728235 -0.026489298716051563193 0.010509127941489518343 0.0089650322888804208354 -0.0055258875060298457901 -0.0013709352608155460189 0.0050745302671845165382 0.019324243095909146573 0.01613431088170272712 0.0025885767027393228809 0.0066361742347835041625 0.0022314450074135405711 -0.006681404395148900742 0.015252382649606613385 0.01202510770804145375 -0.011526623626134815601 -0.0089968566807257649431 0.0014719898717660711597 -0.00041713853452431831105 -2.771944505695356773e-05 -0.0037795999423066743358 -0.0077269815959212870965 -0.0099141343363440444236 -0.0050880597821067480996 0.00094992464383219675159 0.0059767056359274171873 -0.010897969359634676612 0.004041791971318065349 -0.0016533095772796636835 0.012822760603393334317 -0.0312846950442256036 -0.003176108647995269825 0.00083912192344727249815 -0.01168558698856722762 -0.020007630559126302755 -0.00019548123235915628327 -0.011058695507354956988 0.017201952922523106038 -0.04816841671773505057 0.022010131367101011535 -0.0088483489445811934482 0.0027206066751053574471 -0.018436063364597351372 -0.015844742729743260812 0.020745115278842346518 0.031009786284102953957 -0.016355207409975570931 0.018173823195486500659 0.0022967697783240971672 -0.0066697102273754332868 -0.0045816728234429893174 0.0036607752490185677119 -0.011285784199826180002 -0.0087417884401751588114 -0.0047525442123182159992 -0.002488298342244574593 0.0080171073478409564383 0.0042299640715365441693 0.0035394482031119254423 0.006625069309020154007 0.0037971839357273029156 -0.00038447430794315387874 -0.00024381269198143232192 -0.0011192892003117340879 5.3250403603121105114e-05 -0.00050798324050273951404 0.0026881422233781850084 -3.9434077098764657799e-05 0.0013914132719977298763 -0.0017270227799662132926 -0.0002174079295685267272 -0.0053042312481453215284 0.034974685002730918071 0.72559003344605788044 0.33277199314265298868 0.50710565399821938559 0.8248210661368209351 0.52669130620389803532 -0.18118767641908323229 0.32235341775103870665 0.05980693174331559725 -0.33186268593638745816 -0.52976274482931706711 0.84810419884567056492 0.29902259507032086239 0.056299235343292390599 -0.576131228162021225 0.75818294759881255107];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 0.0149076572882655;
y1_step1.xoffset = -64.3649297385139;

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
