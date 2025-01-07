{ pkgs ? import <nixpkgs> {}}:


pkgs.mkShell
{
  nativeBuildInputs = with pkgs; [
    virtualenv
    python312Full
    python312Packages.virtualenv
    python312Packages.tkinter
    stdenv.cc.cc.lib
    ninja
    cmake
    zlib
  ];

  
  shellHook   = ''
    virtualenv -p python3.12 .venv
    # skip, if already activated
    [ -f .venv/bin/activate ] &&
    source .venv/bin/activate &&
    pip install -r requirements.txt &&
    pip install -e .
  '';
}
