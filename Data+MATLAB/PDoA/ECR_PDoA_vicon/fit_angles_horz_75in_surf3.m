function [fitresult, gof] = fit_angles_horz_75in_cubic(pdoa_horz_75in, pdoa_vert_75in, angles_horz_deg_adj_75in)
%CREATEFIT(PDOA_HORZ_75IN,PDOA_VERT_75IN,ANGLES_HORZ_DEG_ADJ_75IN)
%  Create a fit.
%
%  Data for 'fit_angles_horz_75in_cubic' fit:
%      X Input : pdoa_horz_75in
%      Y Input : pdoa_vert_75in
%      Z Output: angles_horz_deg_adj_75in
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 15-May-2020 19:23:51


%% Fit: 'fit_angles_horz_75in_cubic'.
[xData, yData, zData] = prepareSurfaceData( pdoa_horz_75in, pdoa_vert_75in, angles_horz_deg_adj_75in );

% Set up fittype and options.
ft = fittype( 'poly33' );

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% % Plot fit with data.
% figure( 'Name', 'fit_angles_horz_75in_cubic' );
% h = plot( fitresult, [xData, yData], zData );
% legend( h, 'fit_angles_horz_75in_cubic', 'angles_horz_deg_adj_75in vs. pdoa_horz_75in, pdoa_vert_75in', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'pdoa_horz_75in', 'Interpreter', 'none' );
% ylabel( 'pdoa_vert_75in', 'Interpreter', 'none' );
% zlabel( 'angles_horz_deg_adj_75in', 'Interpreter', 'none' );
% grid on
% view( -20.0, 17.1 );


