function [fitresult, gof] = fit_angles_vert_75in_linear(pdoa_horz_75in, pdoa_vert_75in, angles_vert_deg_75in)
%CREATEFIT(PDOA_HORZ_75IN,PDOA_VERT_75IN,ANGLES_VERT_DEG_75IN)
%  Create a fit.
%
%  Data for 'fit_angles_vert_75in_linear' fit:
%      X Input : pdoa_horz_75in
%      Y Input : pdoa_vert_75in
%      Z Output: angles_vert_deg_75in
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 15-May-2020 19:34:14


%% Fit: 'fit_angles_vert_75in_linear'.
[xData, yData, zData] = prepareSurfaceData( pdoa_horz_75in, pdoa_vert_75in, angles_vert_deg_75in );

% Set up fittype and options.
ft = fittype( 'poly11' );

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% % Plot fit with data.
% figure( 'Name', 'fit_angles_vert_75in_linear' );
% h = plot( fitresult, [xData, yData], zData );
% legend( h, 'fit_angles_vert_75in_linear', 'angles_vert_deg_75in vs. pdoa_horz_75in, pdoa_vert_75in', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'pdoa_horz_75in', 'Interpreter', 'none' );
% ylabel( 'pdoa_vert_75in', 'Interpreter', 'none' );
% zlabel( 'angles_vert_deg_75in', 'Interpreter', 'none' );
% grid on
% view( -56.2, -2.1 );

