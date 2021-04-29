%% Data segmenting (automate this next time...although excel made it not terrible)

%% Dataset
indices = data_raw(:,1) == 0; %index markers
rows = find(indices);
count = nnz(indices);

point001	=	data_raw	(	rows(	1	) + 1	:	rows(	2	) - 1	,:);
point002	=	data_raw	(	rows(	2	) + 1	:	rows(	3	) - 1	,:);
point003	=	data_raw	(	rows(	3	) + 1	:	rows(	4	) - 1	,:);
point004	=	data_raw	(	rows(	4	) + 1	:	rows(	5	) - 1	,:);
point005	=	data_raw	(	rows(	5	) + 1	:	rows(	6	) - 1	,:);
point006	=	data_raw	(	rows(	6	) + 1	:	rows(	7	) - 1	,:);
point007	=	data_raw	(	rows(	7	) + 1	:	rows(	8	) - 1	,:);
point008	=	data_raw	(	rows(	8	) + 1	:	rows(	9	) - 1	,:);
point009	=	data_raw	(	rows(	9	) + 1	:	rows(	10	) - 1	,:);
point010	=	data_raw	(	rows(	10	) + 1	:	rows(	11	) - 1	,:);
point011	=	data_raw	(	rows(	11	) + 1	:	rows(	12	) - 1	,:);
point012	=	data_raw	(	rows(	12	) + 1	:	rows(	13	) - 1	,:);
point013	=	data_raw	(	rows(	13	) + 1	:	rows(	14	) - 1	,:);
point014	=	data_raw	(	rows(	14	) + 1	:	rows(	15	) - 1	,:);
point015	=	data_raw	(	rows(	15	) + 1	:	rows(	16	) - 1	,:);
point016	=	data_raw	(	rows(	16	) + 1	:	rows(	17	) - 1	,:);
point017	=	data_raw	(	rows(	17	) + 1	:	rows(	18	) - 1	,:);
point018	=	data_raw	(	rows(	18	) + 1	:	rows(	19	) - 1	,:);
point019	=	data_raw	(	rows(	19	) + 1	:	rows(	20	) - 1	,:);
point020	=	data_raw	(	rows(	20	) + 1	:	rows(	21	) - 1	,:);
point021	=	data_raw	(	rows(	21	) + 1	:	rows(	22	) - 1	,:);
point022	=	data_raw	(	rows(	22	) + 1	:	rows(	23	) - 1	,:);
point023	=	data_raw	(	rows(	23	) + 1	:	rows(	24	) - 1	,:);
point024	=	data_raw	(	rows(	24	) + 1	:	rows(	25	) - 1	,:);
point025	=	data_raw	(	rows(	25	) + 1	:	rows(	26	) - 1	,:);
point026	=	data_raw	(	rows(	26	) + 1	:	rows(	27	) - 1	,:);
point027	=	data_raw	(	rows(	27	) + 1	:	rows(	28	) - 1	,:);
point028	=	data_raw	(	rows(	28	) + 1	:	rows(	29	) - 1	,:);
point029	=	data_raw	(	rows(	29	) + 1	:	rows(	30	) - 1	,:);
point030	=	data_raw	(	rows(	30	) + 1	:	rows(	31	) - 1	,:);
point031	=	data_raw	(	rows(	31	) + 1	:	rows(	32	) - 1	,:);
point032	=	data_raw	(	rows(	32	) + 1	:	rows(	33	) - 1	,:);
point033	=	data_raw	(	rows(	33	) + 1	:	rows(	34	) - 1	,:);
point034	=	data_raw	(	rows(	34	) + 1	:	rows(	35	) - 1	,:);
point035	=	data_raw	(	rows(	35	) + 1	:	rows(	36	) - 1	,:);
point036	=	data_raw	(	rows(	36	) + 1	:	rows(	37	) - 1	,:);
point037	=	data_raw	(	rows(	37	) + 1	:	rows(	38	) - 1	,:);
point038	=	data_raw	(	rows(	38	) + 1	:	rows(	39	) - 1	,:);
point039	=	data_raw	(	rows(	39	) + 1	:	rows(	40	) - 1	,:);
point040	=	data_raw	(	rows(	40	) + 1	:	rows(	41	) - 1	,:);
point041	=	data_raw	(	rows(	41	) + 1	:	rows(	42	) - 1	,:);
point042	=	data_raw	(	rows(	42	) + 1	:	rows(	43	) - 1	,:);
point043	=	data_raw	(	rows(	43	) + 1	:	rows(	44	) - 1	,:);
point044	=	data_raw	(	rows(	44	) + 1	:	length(data_raw)	,:);

%
pointStats = [
mean(	point001	)	,	std(	point001	);
mean(	point002	)	,	std(	point002	);
mean(	point003	)	,	std(	point003	);
mean(	point004	)	,	std(	point004	);
mean(	point005	)	,	std(	point005	);
mean(	point006	)	,	std(	point006	);
mean(	point007	)	,	std(	point007	);
mean(	point008	)	,	std(	point008	);
mean(	point009	)	,	std(	point009	);
mean(	point010	)	,	std(	point010	);
mean(	point011	)	,	std(	point011	);
mean(	point012	)	,	std(	point012	);
mean(	point013	)	,	std(	point013	);
mean(	point014	)	,	std(	point014	);
mean(	point015	)	,	std(	point015	);
mean(	point016	)	,	std(	point016	);
mean(	point017	)	,	std(	point017	);
mean(	point018	)	,	std(	point018	);
mean(	point019	)	,	std(	point019	);
mean(	point020	)	,	std(	point020	);
mean(	point021	)	,	std(	point021	);
mean(	point022	)	,	std(	point022	);
mean(	point023	)	,	std(	point023	);
mean(	point024	)	,	std(	point024	);
mean(	point025	)	,	std(	point025	);
mean(	point026	)	,	std(	point026	);
mean(	point027	)	,	std(	point027	);
mean(	point028	)	,	std(	point028	);
mean(	point029	)	,	std(	point029	);
mean(	point030	)	,	std(	point030	);
mean(	point031	)	,	std(	point031	);
mean(	point032	)	,	std(	point032	);
mean(	point033	)	,	std(	point033	);
mean(	point034	)	,	std(	point034	);
mean(	point035	)	,	std(	point035	);
mean(	point036	)	,	std(	point036	);
mean(	point037	)	,	std(	point037	);
mean(	point038	)	,	std(	point038	);
mean(	point039	)	,	std(	point039	);
mean(	point040	)	,	std(	point040	);
mean(	point041	)	,	std(	point041	);
mean(	point042	)	,	std(	point042	);
mean(	point043	)	,	std(	point043	);
mean(	point044	)	,	std(	point044	);
];


clearvars point* -except pointStats*