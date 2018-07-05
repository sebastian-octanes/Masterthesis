\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{like}[2016/11/24 v. 1.0 Dokumentenklasse fuer DA/BA]

%% optionen
\DeclareOption{12pt}{
  \PassOptionsToClass{\CurrentOption}{scrbook}
}

%% Fallback
\DeclareOption*{
  \ClassWarning{like}{Unknown option '\CurrentOption'}
}

%% Execute default options
\ExecuteOptions{a4paper, fontsize=12pt,oneside,openany,DIV12,BCOR10mm,titlepage,numbers=noenddot,parskip=half}

%% Process given options
\ProcessOptions\relax

%% Load base
\LoadClass[a4paper, fontsize=12pt,oneside,openany,DIV12,BCOR10mm,titlepage,numbers=noenddot,parskip=half]{scrbook}



%----------------------------------------
%   Seitendefinition,-Einstellungen:
%-----------------------------------------

\RequirePackage{geometry}

 \setlength{\textwidth}{15cm} 
%\setlength{\textwidth}{15.5cm}
\setlength{\textheight}{21cm}
%\setlength{\topmargin}{5mm}
\setlength{\parindent}{0cm}
\setlength{\headheight}{1cm}
\setlength{\headsep}{1cm}
%%%%%%%%%%neu 23.11.17%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\setlength{\topmargin}{-1cm} % Abstand 2.Seite nach oben

\RequirePackage{setspace}   %Zeilenabstand
 %\onehalfspacing         % 1.5 zeilig
\setstretch{1,15}
%%%%%%%%%%neu 23.11.17%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\RequirePackage{mathptmx}
%-------------------------------------
%   Umlaute -  Schrift - Sprache  
%-------------------------------------
%Überschriften - Schriftart -  Times:

\addtokomafont{disposition}{\rmfamily}%Schrift und Sprache:
\RequirePackage{lmodern}
\RequirePackage[T1]{fontenc}   % 8-Bit-Codierung der Fonts verwenden 
\RequirePackage{mathptmx} %Times-New-Roman-artige Schriftart
%\RequirePackage[ngerman,english]{babel}  %Rechtschreibung
%\RequirePackage[english,ngerman]{babel}
\RequirePackage[ngerman]{babel}
%\RequirePackage[english]{babel}  %Rechtschreibung
%\RequirePackage[ngerman]{babel}  %Rechtschreibung
\RequirePackage[utf8]{inputenc} 
%\RequirePackage[utf8]{inputenc} 
\RequirePackage{times}
%Apple-Nutzer sollten anstelle von \usepackage[utf8]{inputenc} das Paket \usepackage[applemac]{inputenc} verwenden


%------------------------------------------------
% Chapter/Section/Subsection-fonts 
%------------------------------------------------

%\RequirePackage{sectsty}
%\chapterfont{\fontfamily{ptm}\fontseries{bf}\fontsize{22pt}{22pt}\selectfont}
%\sectionfont{\fontfamily{ptm}\fontseries{bfseries}\fontsize{18pt}{18pt}\selectfont}
%\subsectionfont{\fontfamily{ptm}\fontseries{bfseries}\fontsize{16pt}{16pt}\selectfont}
%\subsubsectionfont{\fontfamily{ptm}\fontseries{bf}\fontsize{14pt}{14pt}\selectfont}

% Warning: Font shape `T1/ptm/bf/n' undefined
%-------------------------------------
%       allgemeine Fonts auf textrm
%-------------------------------------
%\renewcommand{\normalsize}{\fontsize{12pt}{16pt}\selectfont

\fontfamily{ptm}\selectfont

% \renewcommand*\chapterformat{\thechapter~~}  
%%-------------------------------------
%       Itemize Umbegungs-änderung
%-------------------------------------
%\renewcommand{\labelitemi}{$\bullet$}
%\renewcommand{\labelitemii}{$\circ$}
\renewcommand{\labelitemiii}{$\circ$}
\renewcommand{\labelitemiv}{$\cdot$}





%------------------------------------------------
%  Linien / Kapitel auf 2. Seite /Folgeseiten
%------------------------------------------------
%\RequirePackage{slantsc}%darstellung von itshape+scshape gleichzeitg funzt aber net
\RequirePackage{ifthen}
%\RequirePackage[automark,headsepline]{scrpage2}   %
%\pagestyle{scrheadings} 
%\clearscrheadfoot 
%\ihead{\headmark                                %Kolumnentitel
%  \vspace{2mm}}                                  % Abstand
%\cfoot[\pagemark]{\pagemark}
%------------------------------------------------
%  Linien / Kapitel auf 2. Seite /Folgeseiten
%------------------------------------------------
\newcommand\headrule{
\vspace{-1cm}
\begin{minipage}{1\textwidth}
\hrule width \hsize \kern 2mm 
\end{minipage}
}

%------------------------------------------------
% %%%% Einstellung 2. Seite laufender Kolumnentitel:
%------------------------------------------------

%------------------------------------------------
%  Linien / Kapitel auf 2. Seite /Folgeseiten
%------------------------------------------------
%%\RequirePackage{slantsc}%darstellung von itshape+scshape gleichzeitg funzt aber net
%\RequirePackage{ifthen}
%\RequirePackage[automark,headsepline]{scrpage2}   %
%\pagestyle{scrheadings} 
%\clearscrheadfoot 
%\ihead{\headmark                                %Kolumnentitel
%  \vspace{2mm}}                                  % Abstand
%\cfoot[\pagemark]{\pagemark}
%
%\setkomafont{pageheadfoot}{%Kopfzeilen in Kapitälchen
%\large\scshape}
% 
%
\RequirePackage[headsepline]{scrpage2}

\pagestyle{scrheadings} 
\automark[chapter]{chapter}
%\ihead{\headmark                                %Kolumnentitel
%  \vspace{2mm}}                                  % Abstand
%\cfoot[\pagemark]{\pagemark}
\clearscrheadfoot 
\ihead{\headmark} %% Im Kolumnentitel steht das Kapitel sowie der jeweilige Abschnitt - solllte kein
% Abschnitt vorhanden sein - nur das Chapter

%\ohead{\ifstr{\rightmark}{\leftmark}{}{\rightmark}}
 % \vskip 0.5cm
\setkomafont{pagehead}{%Kopfzeilen in Kapitälchen
\normalsize\scshape}

\setkomafont{pagefoot}{%Kopfzeilen in Kapitälchen
\normalsize\scshape}    
%\setkomafont{pageheadfoot}{%Kopfzeilen in Kapitälchen
%\normalsize\scshape}                              %

%Seitenzahl mit:
\cfoot[\pagemark]{\pagemark}
\makeatletter

% Einstellung dass /chapter die 2 Linien hat auf den chapteranfangsseiten:

\def\@makeschapterhead#1{   %  \chapter* command makeschapterhead)
  \vspace*{-110pt}                     % Space at top of page.
  { \parindent 0pt \raggedright\vspace*{10pt}
 \parskip 10pt\rule{\textwidth}{0.5pt}\vspace*{6mm} 
\LARGE\bfseries{#1}\par
  \parskip 18pt\rule{\textwidth}{0.5pt}                   
}   \vskip 20pt% Space between title and text.
   }

\def\@makechapterhead#1{             % Heading for \chapter nummeriert
   \vspace*{-110pt}                     % Space at top of page.
  { \parindent 0pt \raggedright\vspace*{10pt}
 \parskip 10pt\rule{\textwidth}{0.5pt}\vspace*{6mm} 
\LARGE\bfseries\thechapter~~#1\par
  \parskip 18pt\rule{\textwidth}{0.5pt}                   
}   \vskip 20pt% Space between title and text.
   }
%
\makeatother      % %Befehle, die @enthalten müssen makeatother enden




%-------------------------------------------
%   Abstände vor und nach Section/subsection
%-------------------------------------------
%

  
\RedeclareSectionCommand[
  beforeskip=0.2\baselineskip,
  afterskip=0.5\baselineskip]{section}
  
  \RedeclareSectionCommand[
  beforeskip=1\baselineskip,
  afterskip=0.4\baselineskip]{subsection}
  
%
% 
%-------------------------------------------
%      Verzeichnisse / Nummerierungtiefe     
%-------------------------------------------

\setcounter{secnumdepth}{4} % Überschriften bis 3.Ebene nummeriert
\setcounter{tocdepth}{3}    % Tiefe des Inhaltsverzeichnisses
\RequirePackage[nottoc,notlot,notlof]{tocbibind} % Einbindung des Literatur- und Abbildungsverzeichnisses ins Inhaltsverzeichnis
\RequirePackage[printonlyused]{acronym} %%% zur Erstellung des Abkürzungsverzeichnis
%\RequirePackage{glossaries} 
%\RequirePackage{makeidx}
%\makeindex



%-------------------------------------------
%      Nummerierung Seiten römisch     
%-------------------------------------------

\pagenumbering{roman}

%-----------------------------------------------
%%  Symbole /Grafiken/ Gleichungen:      
%-----------------------------------------------

\RequirePackage{textcomp} %Für extra Zeichen
\RequirePackage{latexsym}
\RequirePackage{amsmath} %zur Nummerierung von Gleichungen
\RequirePackage{amssymb} % Gleichungen
\RequirePackage[right]{eurosym} %Eurosymbol \EUR{Zahl}, oder Zahl \euro{}
\RequirePackage{graphicx}
\RequirePackage{float}
\RequirePackage{floatflt}
\RequirePackage{multirow}
\RequirePackage[usenames,dvipsnames]{xcolor}
\RequirePackage{mathtools}   % lädt »amsmath«
\definecolor{color0}{rgb}{0,0,0}% black
\definecolor{color1}{rgb}{0.22,0.45,0.70}% light blue
\definecolor{color2}{rgb}{0.45,0.45,0.45}% dark grey

%----------------------------------------------------
% Kapitelnumerierung der Abbildungen und Tabellen
%----------------------------------------------------
\RequirePackage{subfigure}

\numberwithin{figure}{chapter} %für die Abb.
\numberwithin{table}{chapter} %für die Tabellen
\numberwithin{equation}{section}   % Nummerierung der Formeln abschnittsweise

%-------------------------------------------
%          weitere Pakete:
%  für Links, Blindtext, Layout Titelseite     
%-------------------------------------------

\RequirePackage{refstyle}
\RequirePackage{lipsum} % Blindtext
\RequirePackage{scrhack}

%\RequirePackage{microtype} %badboxes sollten veschwinden -geht aber net
%\hyphenpenalty=750\RequirePackage{caption} % Gleichungen
%
%\pretolerance=10000
%\tolerance=50000
%\emergencystretch=10pt

%____________________________________
%Environments neu
%\newenvironment{acknowledgments}{
%\chapter*{\acknowledgmentname}
%%\addtocontents{toc}{\vspace{12pt}}
%\addcontentsline{toc}{chapter}{\hspace{-1.5em} \MakeUppercase{\acknowledgmentname}}
%\vspace{5ex}
%}{}
%


%\newenvironment{abstract}{%
 %        \chapter*{\abstractname}}%
%-------------------------------------------
%               Bildunterschriften   
%-------------------------------------------
\RequirePackage{ragged2e}
\RequirePackage{caption}
%\captionsetup{format=plain,indention=2.3cm,font=small,labelfont=bf,labelsep=colon,justification=RaggedRight}
%\captionsetup{font=small,labelfont=bf} %
\captionsetup{format=hang,justification=raggedright}
\captionsetup{font=small,labelfont=bf}
%Sollte es wider erwarten zu komischen Trennungen kommen - kann man die Befehle 
%\sloppy % weniger Worttrennungen, größere Wortabstände
%\fussy % viele Worttrennungen, "schönere" Wortabstände
%einfügen 

%-------------------------------------------
%               Links    
%-------------------------------------------
\RequirePackage[colorlinks]{hyperref}
\hypersetup{linkcolor=color0,citecolor=color0,filecolor=black,urlcolor=color0} 
%bilatex alternative:
%\usepackage[backend=biber,style=alphabetic,sorting=nyt,backref=true]{biblatex}
%\addbibresource{literatur}

%\bibliographystyle{IEEEtran-de}
%\addcontentsline{toc}{chapter}{Bibliography}
%\bibliographystyle{unsrt}
%\bibliographystyle{ieeetran}
%\bibliography{literatur}
%%

%-------------------------------------------
%            no badbox-Warnings!!!!:    
%-------------------------------------------

%\flushbottom
\def\sloppy{\tolerance=10000000\hfuzz=\maxdimen\vfuzz=\maxdimen}
\vbadness=18000
\hbadness=20000
%\sloppy
% no badbox-Warnings- Ende!!!!

%-------------------------------------------
%  Hurenkinder un Schusterjungen verhindern  
%-------------------------------------------
%Hurenkinder un Schusterjungen verhindern
\clubpenalty = 10000
\widowpenalty = 10000 
\displaywidowpenalty = 10000

%-------------------------------------------
%     Trennen von Inline Formeln unterbinden  
%-------------------------------------------

\relpenalty=9999
\binoppenalty=9999

%---------------------------------------------------------------------------


\DeclareOption*{
 \PassOptionsToClass{\CurrentOption}{like}} \ProcessOptions\relax
 
 
