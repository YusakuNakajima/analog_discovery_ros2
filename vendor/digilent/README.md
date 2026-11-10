# Digilent Adept Runtime + WaveForms

The unmodified Linux x86_64 `.deb` packages from Digilent are attached to this
repository's GitHub Release
[`digilent-debs-adept2.30.1-waveforms3.25.1`](https://github.com/YusakuNakajima/analog_discovery_ros2/releases/tag/digilent-debs-adept2.30.1-waveforms3.25.1),
so that the setup needs no browser download (files.digilent.com is behind a
Cloudflare challenge). They are not committed to git, to keep the clone small.
`scripts/install_waveforms.sh` downloads them into this directory (git-ignored),
checks their sha256 and installs them.

| File | Version | sha256 | Source |
| --- | --- | --- | --- |
| `digilent.adept.runtime_2.30.1_amd64.deb` | 2.30.1 | `e5e51d2640c2ff34ef3b436f3bdf37838120b15160d73dfbab82e90773b6b372` | https://digilent.com/reference/software/adept/runtime |
| `digilent.waveforms_3.25.1_amd64.deb` | 3.25.1 | `d2979aab726c9202a48a1c5d2b314531513171c0b62fa2f2a2edcd29202727d3` | https://digilent.com/reference/software/waveforms/waveforms-3/start |

License: [LICENSE](LICENSE) (the same text ships inside both packages, and it is
attached to the Release too). It permits copying and redistribution of the
unmodified software, for use only with Digilent hardware, with the notice kept.
Copyright (c) 2022 Digilent Inc.

To move to a newer version: create a new Release with both files and LICENSE,
then update `RELEASE_URL` and `KNOWN_SHA256` in `scripts/install_waveforms.sh`
and this table.
