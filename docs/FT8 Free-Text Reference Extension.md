# FTRX: FT8 Free-Text Reference Extension (v0.4)

FTRX is a lightweight convention to carry a short “reference” (e.g., SOTA/POTA/etc.) using **FT8 Free Text**, while staying compatible with existing WSJT-X workflows.

This is an **application-layer** convention only (no FT8 protocol changes).

---

## 1. Message Format

Let the transmitted FT8 Free Text be `T` with length `L`.

FTRX is defined as:

- **Tail (mandatory):** the **last 4 characters** of `T` are `[DX2][MY2]`
- **Reference:** everything before the last 4 characters is `REF`

So:

`T = REF + DX2 + MY2`

Constraints:

- `L` is **5..13**
- `REF` length is **1..9**
- `REF` may contain **spaces**
- `REF` may use any character from the FT8 free-text **(A-Z 0-9 /+-.?)**
- `DX2` and `MY2` are 2-character tags derived from callsigns (see §2)

> Parsing note: Because receivers validate based on the **last 4 characters**, allowing spaces inside `REF` does not create ambiguity.

---

## 2. Callsign Tag Derivation (DX2 / MY2)

To derive the 2-character tag from a callsign:

1. If the callsign contains `/`, take the **left token** as the base call  
   - Example: `AG6AQ/KH6` → base `AG6AQ`
2. DXpedition formatting note: write `KH7Z/KH1` (not `KH1/KH7Z`) so the left-token rule selects the operator call; otherwise the derived tag may be incorrect.
3. Remove all digits `0–9` from the base call
4. Take the **last 2 characters** of what remains  
   - If fewer than 2 characters remain, the tag is invalid

---

## 3. Validation (Receiver)

A received Free Text `T` is accepted as valid FTRX only if:

1. `L` is **5..13**
2. Extract:
   - `DX2 = T[L-4 .. L-3]`
   - `MY2 = T[L-2 .. L-1]`
   - `REF = T[0 .. L-5]` (length 1..9)
3. Compute:
   - `DX2_expected` from the **other station’s** callsign
   - `MY2_expected` from **my** callsign
4. Require exact match:
   - `DX2 == DX2_expected` **and**
   - `MY2 == MY2_expected`

If any check fails, treat it as ordinary Free Text.

---

## 4. Semantics

FTRX is only used as a **sign-off equivalent**.

- FTRX may replace whichever sign-off message is appropriate for **my station** at that step:
  - `RR73` **or**
  - `73`

Conceptually:

> **FTRX = (RR73 or 73, whichever I should send now) + reference REF**

Only **one reference** should be sent per QSO per station.

---

## 5. Mini-FT8 Sequence Adaptation

Mini-FT8 integrates FTRX with two user settings for backward compatibility with WSJT-X.

### 5.1 Settings

- `Ref` (string)
  - length **1..9** (empty disables)
  - spaces allowed
- `Send_Ref` (enum): `0 | 1 | 2`
  - `0`: do not send FTRX
  - `1`: replace **one** sign-off (`RR73` or `73`) with FTRX
  - `2`: replace up to **two** sign-offs with FTRX

### 5.2 Rule

When Mini-FT8 is about to transmit a **sign-off** (`RR73` or `73`):

- If `Send_Ref > 0`, `Ref` is set, and `ref_tx_count < Send_Ref`:
  - transmit `T = Ref + DX2 + MY2` (FTRX)
  - increment `ref_tx_count`
- Else:
  - transmit the normal sign-off message (`RR73` / `73`) per standard FT8 sequencing

### 5.3 Fallback

After 1–2 FTRX attempts (per `Send_Ref`), if the other station does not converge cleanly, Mini-FT8 **falls back to normal sign-off** messages to complete the QSO in standard WSJT-X style.

---

## 6. Examples (AG6AQ at W6/CC-052, N6HAN at W6/CC-072)

Tags (digits removed, last 2 chars):
- `AG6AQ` → `AQ`
- `N6HAN` → `AN`

### Example A: AG6AQ sends FTRX as final sign-off
- N6HAN → `AG6AQ N6HAN -10`
- AG6AQ → `N6HAN AG6AQ R-12`
- N6HAN → `AG6AQ N6HAN RR73`
- AG6AQ → `W6/CC-052ANAQ`  *(FTRX replaces AG6AQ’s sign-off here)*

### Example B: N6HAN sends FTRX as sign-off (replacing RR73)
- N6HAN → `AG6AQ N6HAN -10`
- AG6AQ → `N6HAN AG6AQ R-12`
- N6HAN → `W6/CC-072AQAN`  *(FTRX replaces N6HAN’s RR73)*
- AG6AQ → `W6/CC-052ANAQ` *(or `N6HAN AG6AQ 73`)*

### Example C: Allowing spaces in REF
- … report exchange …
- N6HAN → `W6/CC072 AQAN`  *(REF contains a space; tail is still `AQAN`)*
- AG6AQ → `W6/CC052 ANAQ` *(or normal `73`)*
- done

---

## Notes / Non-goals

- FTRX does not attempt to standardize reference namespaces (SOTA vs POTA, etc.) beyond “up to 9 characters”.
- FTRX is designed to be robust and backward compatible: stations that do not implement it will simply see Free Text.
