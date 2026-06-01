%% VUMIFPSbakalaurinis.cls – „Tik svajotojai-romantikai gali pagerinti nepakeičiamą pasaulį“
%% Created from   „Atviras Kodas Lietuvai“  2015
%
%% Latest version available here: https://github.com/LIKS/bachelor_thesis_template_vu_mif_se
%% Any suggestions or contributions are welcome!  Also see: https://latex.liks.lt!
%
%% License CC BY 4.0 (see more at http://creativecommons.org/licenses/by/4.0/)
% You are free to (a) share - copy and redistribute the material in any medium
% or format (b) adapt - remix, transform, and build upon the material for any
% purpose, even commercially. As long as you give appropriate credit, provide a
% link to the license, and indicate if changes were made.


% Bakalauro baigiamojo darbo apiforminimas pagal Vilniaus universiteto
% Matematikos ir informatikos fakulteto reikalavimus
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{VUMIFTemplateClass}[2024/03/21 work group template class]
\LoadClass[12pt, a4paper]{article}
\RequirePackage{LTPlius}  % Nustatymų sulietuvinimas

% Darbo kalba gali būti anglų
\DeclareOption{english}{%
    \def\@ThesisEng{}%
    \setmainlanguage{english}%
    \setotherlanguage{lithuanian}%
}
\DeclareOption{signatureplaces}{%
    \def\@addsignatureplaces%
}
\ProcessOptions\relax

\RequirePackage{fontspec} % Šrifto pasirinkimui
\setmainfont[
    Path=fonts/Palemonas3.2.05/,
    Ligatures=TeX,
    Extension=.ttf,
    UprightFont=*-nm,
    BoldFont=*-bd,
    ItalicFont=*-it,
    BoldItalicFont=*-bi,
]{Palem3.2.05}

\RequirePackage{setspace} % Intervalas tarp eilučių
\RequirePackage[
    left=3cm,
    top=2cm,
    right=1.5cm,
    bottom=2cm,
    footskip=1.5cm,
]{geometry}  % Lapo paraštės
% Tolygiai paskirsto tekstą nuo kairės iki dešinės paraštės
% \sloppy alternatyva
\tolerance 1414
\hbadness 1414
\emergencystretch 1.5em
\hfuzz 0.3pt
\widowpenalty=10000
\vfuzz \hfuzz

\RequirePackage[titles]{tocloft} % Turinio eilučių išlygiavimo nustatymui
\renewcommand{\cftsecfont}{\normalfont}
\renewcommand{\cftsecpagefont}{\normalfont}
\renewcommand{\cftsecleader}{\cftdotfill{\cftdotsep}}
\renewcommand{\tableofcontents}{%
    \section*{Turinys}%
    \@starttoc{toc}%
}
\RequirePackage{fancyhdr} % Puslapio numerio vietos nustatymui

% Nustatoma puslapio numerio vieta puslapyje
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footrulewidth}{0pt}
\fancyhf{}
\fancyfoot[R]{\thepage} % Numeris bus puslapio apačioje dešinėje
\pagestyle{fancy}

\RequirePackage{caption}

% Informacijos tituliniame puslapyje kintamieji
\newcommand{\studijuprograma}[1]{\def\@studijuprograma{#1}}
\newcommand{\darbotipas}[1]{\def\@darbotipas{#1}}
\newcommand{\darbopavadinimas}[1]{\def\@darbopavadinimas{#1}}
\newcommand{\darbopavadinimasantras}[1]{\def\@darbopavadinimasantras{#1}}
\newcommand{\autorius}[1]{\def\@autorius{#1}}
\newcommand{\antrasautorius}[1]{\def\@antrasautorius{#1}}
\newcommand{\vadovas}[1]{\def\@vadovas{#1}}
\newcommand{\moksliniskonsultantas}[1]{\def\@moksliniskonsultantas{#1}}
\newcommand{\recenzentas}[1]{\def\@recenzentas{#1}}

% Pirma pastraipos eilutė atitraukiama 1 cm
\setlength{\parindent}{1cm}

% Tarpai tarp eilučių -- 1.5 cm
\setstretch{1.5}

% Algoritmo pavadinimas
\RequirePackage{algorithm}
\makeatletter
\renewcommand{\ALG@name}{Algoritmas}
\makeatother

% float paketas turi būti užkraunamas anksčiau nei hyperref
\RequirePackage{float}

% Nuorodos turinyje ir kitur padaromos aktyviomis
\RequirePackage[%
    hidelinks,
    linktoc=all,
    pdflang={\@ifundefined{@ThesisEng}{lt}{en-US}},
    bookmarksnumbered=true,
]{hyperref}

% Paimta iš https://tex.stackexchange.com/a/330535
\RequirePackage{bookmark}
\newcommand*{\sectionbookmark}[1][]{%
  \bookmark[%
    level=section,%
    dest=\@currentHref,%
    #1%
  ]%
}

% Skyriaus pavadinimas, turinyje neturintis numerio
\newcommand{\sectionnonum}[1]{%
    \section*{#1}%
    \addcontentsline{toc}{section}{#1}%
}

\newcommand{\sectionnonumtoc}[2]{%
    \section*{#1}%
    \addcontentsline{toc}{section}{#2}%
}

% Skyriaus pavadinimas, neturintis numerio bei neįeinantis į turinį
\newcommand{\sectionnonumnocontent}[1]{%
    \section*{#1}%
    \sectionbookmark{#1}% Sukuriame PDF nuorodą
}

% Nustatomas bibliografijos stilius
\RequirePackage[
    alldates=iso,
    seconds=true,
    % Iš publikavimo datos rodomi tik metai
    date=year,
    % pakeičiame žodžių skaidymą skiemenimis, kai langid nėra lithuanian
    autolang=hyphen,
    backend=biber,
    eprint=false,
    sortcites=true,
    sorting=anyt,
    giveninits=true,
    % nustatomas citavimo stilius
    %style=numeric, % skaitinis numeravimas
    style=alphabetic, % raidinis numeravimas
    abbreviate=false,
    minalphanames=3,
    maxalphanames=3,
    maxnames=9,
    minnames=4,
]{biblatex}

\DefineBibliographyStrings{lithuanian}{%
    % Literatūros sąrašas.
    in = {iš},
    % Terminai, vartojami biblatex-iso690 bibliografijos stiliuje.
    % Žr. https://github.com/michal-h21/biblatex-iso690/wiki/Translation-Guideline
    %urlalso = {prieiga per internetą},
    urlfrom = {prieiga per internetą},
    urlseen = {žiūrėta},
    % Žr. https://terminai.vlkk.lt/konsultacijos/1986-on-line-prijungtas-prijungtis
    %online = {internetinis}
}

% Pliuso ženklas, kuris rodomas esant >3 autoriams, sumažinamas ir pakeliamas
\renewcommand*{\labelalphaothers}{\textsuperscript{+}}


% Biblatex-iso690 pakeitimai

% DOI laukas turėtų spausdinti pilną URL, ne tik identifikatorių. Pakeista iš
% https://github.com/michal-h21/biblatex-iso690/blob/7ae03dbdff1f187cfead57c5108a73a6b5021f68/iso.bbx#L377
\DeclareFieldFormat{doi}{\addcolon\space\url{https://doi.org/#1}}

% Autorius rašomas kaip Vardas Pavardė, ne Pavardė, Vardas
\DeclareNameAlias{default}{given-family}
% Skirtukai tarp autorių padaromi kableliais
\DeclareDelimFormat{multinamedelim}{\addcomma\space}
\DeclareDelimFormat{finalnamedelim}{\addcomma\space}
% Nepaverčiame pavardžių didžiosiomis raidėmis
%\renewcommand{\familynameformat}[1]{#1}

% Padarome, kad nepridėtų [internetinis] prie šaltinių, turinčių
% url lauką. Nukopijuota ir pakeista iš
% https://github.com/michal-h21/biblatex-iso690/blob/7ae03dbdff1f187cfead57c5108a73a6b5021f68/iso.bbx#L721-L750
\renewbibmacro*{medium-type}{%
  \iffieldundef{howpublished}{}% Don't print anything
    {\ifboolexpr{
        test {\iffieldequalstr{howpublished}{online}}
        and not test {\iftoggle{bbx:url}}
        and not test {\ifentrytype{online}}
      }
        {}% Don't print 'howpublished' field
        {\printfield{howpublished}}}%
}

% Nutildome įspėjimus dėl neegzistuojančių failų.
\usepackage{silence}
\WarningFilter{biblatex}{File 'american-iso.lbx' not found!}
\WarningFilter{biblatex}{File 'lithuanian-iso.lbx' not found!}


% TODO: pašalinti csquotes.cfg failą 2025 metais, nes csquotes
% paketas pridėjo lietuvišką kabučių stilių 2023-01-24, tad nebereikės
% specialios konfigūracijos, kai nauja versija bus plačiai prieinama
\RequirePackage[autostyle=true]{csquotes}

\RequirePackage{amsthm}

\newtheoremstyle{customStyle}
{}{}{}
{\parindent}
{\bfseries}{. }{}{}

\swapnumbers

\theoremstyle{plain}
\newtheorem{theorem}{Teorema}[subsection]
\newtheorem{lemma}[theorem]{Lema}
\newtheorem{corollary}[theorem]{Išvada}
\newtheorem{proposition}[theorem]{Teiginys}

\theoremstyle{definition}
\newtheorem{definition}[theorem]{Apibrėžimas}
\newtheorem{remark}[theorem]{Pastaba}
\newtheorem{example}[theorem]{Pavyzdys}

%lietuviški aplinkų Proof, Figure, Table pavadinimai
\addto\captionenglish{\renewcommand{\proofname}{Įrodymas}}

\renewcommand{\figurename}{}
\captionsetup*[table]{name=lentelė.,labelsep=space, labelfont=bf, font=it}
\captionsetup*[figure]{labelsep=space, labelfont=bf, font=it}
