function [Y,Xf,Af] = ANN_60_100n(X,~,~)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 02-May-2020 01:04:53.
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
b1 = [-13.81738824259474363;-17.570527637782230812;-14.720026081927743533;2.6550074053032917121;12.697775730569114572;-13.860453655268338835;-5.6380252267971382452;2.3966629257908835804;-6.3737901290492455786;8.5103559766441723156;-1.4727828312326269877;0.91926951901600639783;-8.2501071694095458042;-2.4182860031272319645;-6.0117485524410749775;-19.455553938464763775;5.6280790844713841636;-6.6137317366738646029;-5.5330347452342056869;-13.376698446615982618;-29.528553736881637093;1.6142803683189239639;14.123149104441008461;-9.2958159533908268912;-4.3460560241772965284;-14.543188142061696411;5.9519508712397826145;-4.497879738989484899;-6.6839311441236377576;0.77582588807947361342;15.042598863883165805;1.6538286144067957384;9.9674523985785938862;-1.6668837662892814766;0.92881634827012660161;8.494910802134175043;4.131317192336893207;0.90728825126296197556;1.001078294289312165;2.9986943750346997106;0.66630278856664193476;2.3221459583397456861;-0.036581907398793411801;-0.93376813932993463219;-3.6304470941844688703;1.3526402072833507706;-2.3170072573261819571;2.6877823872132169392;-1.5599697321575276554;0.69580902781239772903;-0.59919008298692322256;0.55106574815604913997;2.3560270270789356317;7.5292844441994155602;1.2783672564599710686;-2.4376165735342825158;2.9968858647783567228;2.9090674921451791235;-1.117864429532602788;2.3118872117530036014;2.3399668848829313639;-5.9874190160836118224;-1.3113050732173017199;-4.6331093491924608685;3.5118365706158178874;4.2920323275721594669;-13.628727889964208586;5.4490826059457786457;5.4219996011045292761;10.041774979813022028;7.5076595122750235944;-6.1247884402400414317;-13.347550107344737569;10.618527267313339735;4.3196361465447123962;13.514760405825306577;-17.430306320590027269;4.1358616960362457604;6.8366810926625340983;6.6649439068593725466;-2.5905351825271463184;-4.7987366412264709226;-6.5453087554217708188;14.766605225642427612;23.014242114488201452;-12.100782977566606036;-3.7259607096794780112;1.8117640342413905419;9.406483003788480346;-27.099228451059836686;1.208032363868126513;-22.752336427065358748;18.396776654152382946;13.912938924250600081;-6.6780865750243618706;29.868059801646520413;-19.132521313252279782;-1.0569931445438636164;7.0594004275487591826;10.32345186655168412];
IW1_1 = [5.1243044469197407409 7.7674247317291822412 -0.61141598981118239209 7.6543517924728501356;-3.1788895785719337717 10.441541569062172456 6.1498214845308130094 9.8612431031270464388;5.6382376018901121384 8.201528276459939093 -0.63511595429288369985 8.0814881386486643322;-2.2075653127493719552 -0.13279365714635663065 -0.65153335002766266104 0.34373282681271505323;0.43156323601544183122 6.4851267361152160618 -4.0755379300984921898 6.8230382069236190645;17.132999460742240672 -5.0191602623977429332 -1.8925082984235552797 11.490371567334356939;-1.1464741484516003389 -2.0582604405199447228 -9.9227859208662660961 5.4816508406276192034;-0.75266547983549469958 1.9576949894823139076 2.4629411486321561853 -0.1672226261820876636;7.7113696951969279425 -0.34384947971913232267 1.8066236379693192404 1.5602167883077016075;-7.185327951004237157 1.3955038869357165865 0.101050975533174478 1.9566303886057749839;0.67499936051027797923 1.0841827037732971739 2.4697957419700165005 0.31248537635418988101;-1.7024585826300133728 1.4797329776825134395 -0.21940637548741842577 1.0118591522654571957;10.203336525893446307 -0.57315660708608928697 2.6880341198029045202 2.3255178269909828614;1.7214870093067788925 2.3664634651950282596 -0.16167850783350160015 1.8580046551325613002;6.0999566603705135392 -3.384496763120865559 1.9843499857392308261 -0.2681640396513795821;14.289654518523811078 11.841580925746770347 26.449292553641303272 5.2695181178388708076;-6.0756828371997055882 3.0296412776730528016 -1.8495556197891083183 0.083951638928701596831;-4.1919522986710591184 -12.89617496030329491 0.61145345099566095204 -14.801678533554301964;6.9778375534958376747 2.6153587228753267802 0.0042163713021612881857 3.6916974043301609143;15.044722768022248616 5.1812049951115213275 4.1879983278725969598 11.907530893694527663;37.93743450429523989 -7.3356003270203604671 4.8002655658793775828 20.298313201547706797;0.61212434370581303433 -1.0593136665433655619 -7.6223679772343553296 0.50423737760253040108;-17.227577783617199714 4.7338087265584558594 1.6468805226550218279 -11.285552955299559486;15.148827968228337326 3.5517312582496747275 0.13944703674349465117 8.1186548638489703933;0.14882883684631389998 -7.4023882094410629051 3.4102049203062581029 -8.5865148893994334145;17.544239719442384029 -4.4919063866312978206 -1.3692125866329343253 11.219277365876395791;-7.4816171735367831275 -2.7410311285524060487 -0.022304422280393611938 -3.9005514328043773276;6.5059610059726500353 -3.6783638074439086729 -0.075059660061790670071 -0.74283309527020069396;10.74881815594681278 -9.2837249593890867061 -5.6423023039374298193 -3.9522671720325495137;-2.1596668856147376303 1.5466688224755629211 -0.23290801189129020643 0.96899429186693253335;-26.477009662171109738 -20.252622142830141172 1.6489911785465465321 -22.571571095146140351;-1.4670950238926456777 -3.1130009587670053328 0.31256078535146841757 -2.2189410032062486522;-16.29360769907182771 -3.6562294547252935928 -0.14990985237930828933 -8.7353098628754217714;1.6059781375518260571 2.8266812853777043557 -0.25858240128881193609 2.1140389343131809063;-3.3043162495710407534 2.4836553551453603994 3.5555248275789770318 -3.16890121523871926;2.0508243422910381604 -11.099927428245207395 0.9743775102940166466 -10.066513568081292718;-1.3481502752973644199 7.7570612613615281816 -6.2740240735406755945 9.7534364792560115376;-7.7169069296387116097 -6.1533463422744150151 0.87584196893165322528 -4.7952665608632552008;-0.64606817386397652481 -0.71015957884171110326 -1.3449853004101990361 -0.13233139654575787336;-8.5969168132258921844 10.667446918485468998 -0.079282460850850014489 5.3579348622903264499;-0.2956622965493841404 0.45151080263650933411 7.135763827959925365 1.4193426362787515682;0.74658215146762196834 -6.8312740393683037965 0.81576484936745874421 -4.1051086643779557406;-3.0854053172199007449 0.3838453025573812516 0.64852692094336217288 0.53440704755009171834;-10.963940177615700833 2.4489819202011675614 -1.213905144620406773 3.8225083657137024495;0.13178522487546323605 -6.1866166276347183128 2.9882075309308202549 -7.4576254718080834394;-8.3795401051450166818 -0.7205619729064620671 -0.41381903998793079236 -0.38269277019023328901;9.6931808551505405092 0.8811288540714262707 0.47688475500988286582 0.67853052098541966064;-0.79848158248356493338 -7.164923627741947243 0.843633766807199037 -4.8661856283892719333;8.2565515155117807922 0.70030040107674118577 0.42644432149286654044 0.43843599289559348486;-0.24482439522794408049 0.39047874100228385785 7.5482202991723221785 1.3616034483763805873;11.930145703405909074 5.1384537139073147571 -1.1046368593728710383 4.0394551267861018573;-11.913778032092004366 -5.2907787049829462234 1.1499273191314891562 -3.9712464805627472053;1.138387263998497323 -6.7908897933427612159 0.60917127416233129988 -4.2543608733985900372;14.421054328974785008 -9.3040750087581258754 -1.2849419807480384481 -13.346286934228588805;3.9837989334831283017 -6.6290202212847173513 9.1648466578033183083 5.1360865310377050719;-0.086694570521464744961 6.9726175908136616144 -0.94940670919881242629 4.3319218737042328726;3.2310019347995919681 -3.2824758457946647816 -9.7898103168326358059 -1.0095066559670848871;-1.3475343815275431236 -7.441889484055243642 0.79852416476381871124 -5.1739698753830474232;-2.9005798049013917783 0.98761737828558826546 2.2790388035674644662 -1.9693077328271650916;-0.87254758327220738323 1.8235686384306795116 2.6000213762550230534 -0.3228361539165398586;7.1509029890371662574 5.2775559924652455379 -0.34402272953047347004 3.3736418408969814919;-7.8280938805469224562 5.3705703620869673642 1.7924924636625954033 2.7144628532820904709;-7.0497694550975698036 3.1320073014778011533 1.3041454055698562531 2.4835325159090082536;-6.7462671155475222307 -4.6370691661999430266 -17.264988675872874779 0.86505799147718331898;-0.38015721267442093012 3.0198856990047691085 -26.713104031107892666 -6.4702875578914698096;6.189057165596333121 4.7603429657870259817 16.106087364930971262 -1.442921109482286024;-16.100691797323136711 11.390177547923032719 3.4394110984816563814 6.6222829452876359468;-3.5336788159820500255 10.203225031882579898 -0.1724452222418206282 7.7113069295717737717;20.558568484597572024 11.033967606888095858 0.22708984067436435716 6.799613727704254984;14.407408051055920595 1.3418405868493050725 1.0005438507374051493 9.1900812530917228571;9.9973619152001571564 -6.6458010729230094782 -2.1108427822053008605 -3.6231553480134319756;-8.575309097800126068 2.8359430043534845645 1.5911312515203686058 0.53469198708179233037;-18.941982958245812796 3.3929427215673615414 1.7988056032341896984 -21.393657109557850049;7.9136206710345415871 -8.6524863956455586589 12.915869338235422603 -3.7235453310829420559;3.8896594325315398955 0.53907427743940494747 3.8755133954924216866 -0.27966602104830895481;18.984378876215544096 -3.1903450756918156905 -1.7531921663816132373 21.34789654428903205;-7.503830169301676456 19.415367758933854248 -9.9245118265765910337 11.821006569882255732;4.5926194245216080958 -1.777570684925677158 -1.0801079444749599467 -0.11560773455372845564;2.6152510990438595506 4.6998798752643438092 -3.2659470868338913441 8.5963651858421918917;2.6643137177232656398 4.6136849602538285708 -3.0225931417492608588 8.4227997951275064281;-2.0570530427852777855 -1.1119328772165102404 -3.3037545695917605038 1.3471523327181651464;-6.4225323389920374595 2.2573686483714388018 1.2116466381695618626 0.44422606192480951215;-6.8991407202606147564 2.418936708222341192 1.3068433243789638887 0.36067021356913997465;15.026597128458252683 12.771661594925699745 5.7478389723535308065 13.659442360743595302;17.25678078064126808 -14.188857530598884438 -4.8930514928367401239 -4.4737229267995690662;-14.097942710767037866 10.928156018593565335 3.1139048056278242349 4.9557175116784550895;-3.3816161219055924647 0.16073123480189480117 1.8993573682574529382 -1.9798810803604467434;-1.3766403801280646668 -2.0840754323211112542 0.15920458883026955554 -1.6064404367976450327;0.82255852294849507444 7.1072987366483646454 0.37658407919055764124 7.2258700516135183634;-5.6959441573912412693 13.57412348137954794 22.911463444453353588 17.243038051549717693;0.89014328964307742886 0.43878265019404583125 -0.30014937206503894407 1.0112852780215804138;-16.813157997748554351 14.123655442131981985 4.755011931826733651 4.5530925811768820921;22.248425600672451452 -7.5919809062992626991 -4.6364345841435188333 -0.47668867877036202874;16.148998426940639916 -5.617297789375857775 -5.0112703470008748141 6.9646402737696595509;-3.9690968744149248693 4.7352219628382581362 -5.9697814714634924727 2.6686708591837149385;3.8157555517289756253 -18.004838815443569899 13.869098427081766545 -14.795498440150923969;-4.0683223351199702122 13.087259846184666756 -1.5029401697122022341 7.7376027227550396859;-0.67015465772005766709 -0.17386314262168706124 0.31720954787228272664 -0.59105631853633266992;4.0762733344476300701 -4.4744527470155386339 6.1910088805010952129 -3.2101341133343974654;14.600696478884554352 1.4307838813778719356 1.008613899276527448 9.42743674387265429];

% Layer 2
b2 = [-1.3468010300174415583;-2.5518080039467263553;-2.2213083699729878617];
LW2_1 = [-3.5108143205320310187 -0.03219529294493636562 3.2816152759984871246 -0.056901735759144200788 -0.04855610517837098683 1.9268324786920811853 -0.01561107837272822027 -0.58193649265079439026 0.1108362161167452814 0.088286433007706832665 -0.012194102979476408033 -0.87960800931439608874 -0.06678820386829141531 2.1335807137873734618 0.366352939039002512 -0.0026101116175035916911 0.34863551906102041578 0.11097653151917508119 -5.5638606333768132473 -0.04598016157110303842 -0.1184235360941004872 -0.02558498288807678489 4.4021057635514440065 2.4197738168602831443 -0.056448639090873165003 2.5203204132770324897 -5.0788496610804099163 -0.071131199652357729457 0.023951677848536499088 0.91563310270810538416 0.12982136677894234866 0.87132675055416264343 2.1785400884129098031 1.8697919900939277138 0.024200571453015101708 0.068603821242861282026 0.00015222479278984073171 0.042736587131102878045 0.052420370415914518958 -0.056258255998095585415 -0.0042454999042808678686 -1.5887194607642189936 0.21021728923989113635 0.0023072497238972120709 0.01902321831077835243 -2.3259695990906852359 1.2061771290279275082 -0.72859156287977644428 -3.5390394378105738404 0.01564303898404709528 -1.1701196760213223858 -1.1290468680459577744 0.90798448419464228287 -0.1135008700109389479 -0.0070208751390855332272 -1.2427387372889953099 -0.0079626446066578503158 0.19793539857593456244 0.12159180756960653869 0.60801089123296336858 0.07583083778971051736 -0.56665694847940339951 -0.037136917153275099401 0.011920212779272649356 -0.0029302461121301442114 0.0090053621216817630185 0.52670470861251528216 0.14187311332267266151 -0.12539991756935436884 -0.30340848496342814888 -0.41813125305480686178 -1.3575263356617335297 -0.070942305451180798226 -0.0078882787191313884223 0.063680560476595204911 -0.078854209258955379469 0.014590134900418794475 0.041904212226374804029 -0.065871763414190750985 0.061017829430509464006 0.030972862074170281338 2.0347432863900682953 -0.25060568145550532471 -0.0015265093162947309703 -1.3846517526643429274 -0.37365854480691151895 0.32759758622949003426 2.9721720357898679943 0.049889851660346769269 0.010143272313849717101 -1.3611359836183263283 -1.3932502543585401078 0.12842178166221984714 0.0075680961511263933686 -0.10202381638300826727 0.010091233579291420189 0.01439179131349190878 -3.2637588404533275366 -0.11152712982922237395 0.30907421817157354482;-0.3736091271098465616 -0.013403847060381311046 0.24370987567258683981 1.0280788997831431519 -0.12125477779239490772 2.7628396531353991428 0.038205059878504525617 -0.77311223944640539152 0.86438245350799181033 -0.090105464099471441197 -0.44901283007413700377 -1.6362387537933924531 -0.5586090026263040853 -4.733642581196936483 1.1508959684623831787 0.0089859229487160401095 1.1116237596443667712 -0.097524818403523588772 -0.15289962293021933104 -0.0062614113661663894569 -0.14308507900422198778 -0.039643259739800228159 6.2028978441392492726 0.79271646524043148929 -0.17122953469613397259 3.509081619476060343 -0.20774027228464361783 -0.19592081872730124359 0.042117343507405771708 1.5623636671532570297 -0.045965852769012043033 -4.306134772038371139 0.77466321914235858337 -6.8896803690265215891 -0.0044817147276761826885 0.096772074349838338447 0.024524381899466013435 0.11824016924629396352 -0.91655988490936801671 -0.064126109717207419036 -0.13695386145109653131 -2.8091621697552069392 0.18037129756706371531 0.0078940768139817620724 0.17881145084660304834 -1.0094093733843720884 0.55460583614002834096 -2.5408602758153238099 -1.5435390638514774331 0.12591758956496376665 0.87761905518321947639 0.81789064531302690941 1.6411840558145518454 -0.075535836201300138315 -0.010411315415372062246 -2.6014965515614623115 0.014520713507248943566 1.3407693473681558949 0.1107294296494237551 0.76956508174465820815 -0.10911736001793548301 -0.56549866275706672791 0.0053814999520196086374 0.12476190245373389165 0.0042529500384787862627 0.13760494842458401155 -0.8566344929847609091 0.16257697236581225564 0.12738858316780740654 0.55612387806124996548 -0.66101815567923383199 -0.14835162888587330454 0.088037072295993834548 0.015010516274364086795 -0.082272999946324698595 0.13829291548168817783 -0.0098044273100219188344 1.2960572960746987725 -0.046462329699716788112 0.052756089833939139033 -0.20727914018904977644 0.54252053431368030267 0.82206377288681331539 0.006849618155497613943 0.92364523394173148052 0.7259383150060693124 0.5462230726367233169 -7.7419566367483696467 0.43697462920177659873 0.0044156020472779691083 -3.2085035711252438695 0.89815632633062225665 -0.071194742508178060492 -0.03345035010617405935 0.09404457866201447469 0.020186646238410568127 0.0059666451989558964841 -6.2151620313971820408 0.092294835459263480093 -0.5010403578141946479;-0.46419538114654324001 0.82785940418683601205 0.87981756799215182774 -4.8617881478009081775 0.13110356576706394538 0.31075594122596739144 -0.507694623017495128 -4.2263343075272228333 -1.2977257469107943422 0.39396959729556207952 5.1118887065666296365 2.5137526054524039765 0.74208971418446978419 2.9407885639211794881 -1.0556263540484136509 -0.14315674170071851501 -0.96103814823093514086 0.047064456195476903066 -5.188710606477796361 -0.058804297289396761927 -0.0064900376081798126504 0.045971004986970188944 1.2901983573404867123 1.7636736088368711872 3.3609886558691046687 1.0194902105211449683 -4.8608967671858325232 0.13398738971565027356 0.054357192557148575895 -2.7178170350404768918 0.091530192575947716405 1.2331860984990032915 1.5364929650583338816 3.3720025184057118572 0.49594182829644095634 -0.28643264590516537282 -0.50200799564943343611 0.051421863646542727289 10.138798124867154371 0.17015114796353472681 3.433524269510287219 -11.398756299316911367 -0.76835941080404290826 0.3384962713514267918 -4.04567106627828732 -1.5925525258312636101 0.81871536928260935095 -19.671617364356489333 -2.4744553472119772408 -3.129828895552579926 -1.408345979420764893 -1.4734288029202571302 2.5234779575056589529 -0.0027546158679451295849 -0.1032588824720732934 -19.312562961318747057 -0.22099702013193164851 9.1240082776199287196 1.3660148930184288218 4.9086127449115011245 -0.19960320135065520719 -5.367889279262444191 -0.61367679925970064492 -1.4643960274703755609 0.22507170444362217809 -1.6105899199022462032 -3.5272464053802448092 -0.078079198620403050812 0.16917396428860170809 5.0104706249245278826 -3.7968481547877837023 0.27245870685279849344 7.6878605922895628666 -0.40026551350418004827 1.1781569025678260143 7.5084692384618305638 0.20306854363187892831 -10.511075596081493444 -3.5111025784550693452 3.5357552949466577452 2.2200762763903298946 -3.9243884617629696265 -6.6306503943642747245 -0.11830305691173127747 -2.8725822888240526432 4.2010157610098319836 1.8633879756335383071 4.3682402450672404015 -0.10713874222412025916 -0.46860819122455787955 -0.74960615645063122248 -2.8980694222124436799 0.37587525321779402532 -0.23724569928012986808 -3.741996698921771447 -0.4169454945560284731 -0.78897464009043227762 -7.9392149388205677241 -3.4622202503835541876 -4.8693813701048735965];

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
