# Potreban moment motora – bez reduktora i sa reduktorom

```bash
# ============================================================================
# POLAZNI PODACI
# ============================================================================

m      = 175 kg            # masa koja se kreće (robot, teret, ploča, kolica, motor...)
a      = 5 m/s²            # ubrzanje koje želimo
otpori = 350 N             # trenje vođica i lanca + rezerva za pomeranje ruke robota

# Sila koju zupčanik mora da da letvi dok kolica ubrzavaju:
F = m · a + otpori = 175 · 5 + 350 = 1 225 N

# Poluprečnik zupčanika r (krak na kome ta sila deluje):
r_z15 = 0,0159 m
r_z20 = 0,0212 m
r_z30 = 0,0318 m

# Moment koji mora da bude na osovini zupčanika:  M_zup = F · r
M_zup_z15 = 1 225 · 0,0159 = 19,5 N·m
M_zup_z20 = 1 225 · 0,0212 = 26,0 N·m
M_zup_z30 = 1 225 · 0,0318 = 39,0 N·m

# Ovo je NAJVEĆI moment koji je potreban, i to samo dok kolica ubrzavaju
# (2 m/s / 5 m/s² = 0,4 s). To se zove VRŠNI moment.
# Kad kolica voze konstantnom brzinom, treba mnogo manje (samo za otpore).

# Motor uz to mora da ubrza i svoj rotor, pa se dodaje ~10 %.


# ============================================================================
# OPCIJA A – BEZ REDUKTORA  (zupčanik je direktno na osovini motora)
# ============================================================================

# Motor sam daje ceo moment zupčanika:
M_motora = M_zup · 1,1

M_motora_z15 = 19,5 · 1,1 = 21 N·m
M_motora_z20 = 26,0 · 1,1 = 29 N·m
M_motora_z30 = 39,0 · 1,1 = 43 N·m

# -> Najmanji moment traži najmanji zupčanik (z15), jer ima najmanji krak r.


# ============================================================================
# OPCIJA B – SA REDUKTOROM
# ============================================================================

# Reduktor sa prenosom i:
#   motor se vrti i puta BRŽE od zupčanika,
#   a moment na zupčaniku je i puta VEĆI od momenta motora (minus 5 % gubitaka).
# Zato motoru treba i puta MANJI moment:
M_motora = M_zup / (i · 0,95) · 1,1

# Prenos i biramo tako da motor na 3000 o/min daje baš 2 m/s:
#   z15 -> i = 3      z20 -> i = 3      z30 -> i = 5

M_motora_z15 = 19,5 / (3 · 0,95) · 1,1 = 19,5 / 2,85 · 1,1 =  7,5 N·m
M_motora_z20 = 26,0 / (3 · 0,95) · 1,1 = 26,0 / 2,85 · 1,1 = 10,0 N·m
M_motora_z30 = 39,0 / (5 · 0,95) · 1,1 = 39,0 / 4,75 · 1,1 =  9,0 N·m


# ============================================================================
# REZIME – potreban VRŠNI moment motora (dok ubrzava 5 m/s²)
# ============================================================================

#                     z15          z20          z30
# bez reduktora     21 N·m       29 N·m       43 N·m
# sa reduktorom    7,5 N·m       10 N·m        9 N·m    (i = 3 / 3 / 5)

# -> Sa reduktorom motoru treba 3–5 puta manji moment.
```
