extensions [ palette ]

breed [ square-stamps square-stamp ]
breed [ spheros sphero ]

spheros-own [
  flockmates                  ; agentset of nearby spheros
  nearest-neighbor            ; used to decide if separation is necessary
  global-nearest-neighbor     ; for NNI

  netlogo-start-pos           ; used to calculate actually covered distance in real-world environment
  netlogo-target-pos          ; used to calculate distance between wanted and actual positions
  target-speed                ; in percent of maximum speed

  real-distance               ; in patch units, used to calibrate target-distance

  odometry-pos                ; transformed data from sensor stream in patch units
  measured-speed              ; transformed data from sensor stream in cm/s
  measured-heading            ; transformed data from sensor stream between 0°-360°

  ;for plot means
  target-to-odometry-distance
  abs-heading-diff

  ; sums of variables for each robot, divide by number of ticks to get mean
  accumulated-distances       ; monitor mean distance over time
  accumulated-headings        ; monitor mean heading difference over time
  accumulated-target-speeds   ; monitor mean target speed over time
  accumulated-measured-speeds ; monitor mean measured speed over time
]

globals [
  target-distance
  theoretical-nn-distance-mean
  R
  accumulated-R
  r-cluster-limit
  r-regular-limit

  stop-run-flag
  tick-timer
  model-timer
]

to setup
  no-display
  ; don't show steps in between

  clean-up

  color-patches
  create-tiles
  set-up-spheros
  set-up-plot-pens

  display

  set target-distance 0.1 ; for first run only, target distance will be 0.1 patch unit

  write-config-file

  set stop-run-flag false

  reset-ticks
end

to start

  ifelse ticks = 0 [
    reset-timer ; sets timer start point to when go was clicked
  ][

    if not mock-robots [
      print (word "tick #" ticks ": " timer " s, waiting for file")
      set tick-timer (tick-timer + timer)
      reset-timer

      ; wait for python command file
      while [not file-exists? "proceed.txt"] [
        ; do nothing
      ]

      file-open "proceed.txt"
      print (word "tick #" ticks ": " timer " s, file found")
      set tick-timer (tick-timer + timer)
      reset-timer
    ]

    foreach sort spheros [ s -> ask s [
      pen-up
    ] ]
    display

    if not mock-robots [
      while [ not file-at-end? ] [
        let command file-read-line ; (foreach (sort spheros) ... set odometry-pos (list x y), ... set measured-heading, ... set measured-speed)
        carefully [
          run command
        ] [
          print (word "skipping bad netlogo read command: " command)
        ]
      ]

      file-close
      file-delete "proceed.txt"
      print (word "tick #" ticks ": " timer " s, finished reading file")
      set tick-timer (tick-timer + timer)
      reset-timer
    ]

    update-and-plot-metrics
    display

    foreach sort spheros [ s -> ask s [
      pen-down
    ] ]
    display

    print (word "tick #" ticks ": " timer " s, finished plotting and toggle updates")
    set tick-timer (tick-timer + timer)
    reset-timer

    print "------------------------------"
    set model-timer (model-timer + tick-timer)
    set tick-timer 0
  ]

  if stop-run-flag [
    ;stops python while file loop
    if file-exists? "config.txt" [
      file-delete "config.txt"
    ]

    ;print spheros and their target speeds
    foreach sort spheros [ s -> ask s [
      print (word "tick #" ticks ": Sphero " (who - 99) ": final target speed:" target-speed " %")
    ]]

    ifelse model-timer < 60 [
      print (word "tick #" ticks ": total model run time: " precision model-timer 3 " s")
    ][
      print (word "tick #" ticks ": total model run time: " precision (model-timer / 60) 3 " min")
    ]

    stop
  ]

  ;end of architecture loop
  tick


  no-display

  ; avoid border or flock
  foreach sort spheros [ s -> ask s [

    ; you can only start avoiding the borders if you can see them!
    let actual-avoid-distance min(list visibility border-avoidance-distance)

    ifelse pxcor < (min-pxcor + 1 + actual-avoid-distance)
    or pycor < (min-pycor + 1 + actual-avoid-distance)
    or pxcor > (max-pxcor - 1 - actual-avoid-distance)
    or pycor > (max-pycor - 1 - actual-avoid-distance) [
      avoid-borders
    ][
      flock
    ]

  ]]

  display

  print (word "tick #" ticks ": " timer " s, flocking finished")
  set tick-timer (tick-timer + timer)
  reset-timer

  send-commands

  print (word "tick #" ticks ": " timer " s, send commands finished")
  set tick-timer (tick-timer + timer)
  reset-timer

  ; before moving forward the virtual environment distance, save the current pos as netlogo start pos
  foreach sort spheros [ s -> ask s [
    set netlogo-start-pos (list xcor ycor)
  ]]

  foreach sort spheros [ s -> ask s [
    forward target-distance
    set size 0.75
    stamp ; stamp agents at target positions so we know directions of lines
    set size 7.5 / scale ; set size back to real-world size
  ]]

  display

  print (word "tick #" ticks ": " timer " s, forward finished")
  set tick-timer (tick-timer + timer)
  reset-timer

  ;also save the forward positions
  foreach sort spheros [ s -> ask s [
    set netlogo-target-pos (list xcor ycor)
  ]]

end



to update-and-plot-metrics
  foreach sort spheros [ s -> ask s [
    ; mocks the data that otherwise would have been set through file-read-line above
    if mock-robots [
      ; random +-1 does not accurately model true offsets
      let odom-x (xcor + (random-float 2) - 1)
      let odom-y (ycor + (random-float 2) - 1)
      set odometry-pos (list odom-x odom-y)

      set measured-heading (heading + ((random-float 2) - 1) * 45)
      set measured-speed (target-speed + ((random-float 2) - 1)) ;todo look at real values to add a factor here for simulated robot
    ]


    ; calculate and plot target-to-odometry-distance
    set target-to-odometry-distance ((calculate-distance netlogo-target-pos odometry-pos) * scale) ; in cm
    set accumulated-distances (accumulated-distances + target-to-odometry-distance) ; used for monitor averaging the distances for all spheros / ticks

    ; absolute heading differences (0-180°)
    set abs-heading-diff abs (subtract-headings measured-heading heading)
    set accumulated-headings (accumulated-headings + abs-heading-diff)



    ; also used below for target-speed adaptations
    ; compare target distance to actual distance to determine if target speed needs to be adapted
    set real-distance (calculate-distance netlogo-start-pos odometry-pos) ; in NetLogo units

    ;target speed
    set accumulated-target-speeds (accumulated-target-speeds + target-speed) ; where do they level out? set that value as the starting value!? may be different based on surface?

    ; measured speeds
    set accumulated-measured-speeds (accumulated-measured-speeds + measured-speed)

    ; for NNI (with target positions, before potential position update)
    set global-nearest-neighbor min-one-of (other spheros) [distance myself]


    ;; for printing name and plotting values
    let current-sphero-pen-name (word "Sphero " (who - 99))

    ;; 4 x toggles

    ;; positions toggle
    ; only update positions if toggle is on
    if write-back-positions [
      let clamped-x clamp-x-coord (item 0 odometry-pos)
      let clamped-y clamp-y-coord (item 1 odometry-pos)

      ifelse clamped-x != (item 0 odometry-pos) or clamped-y != (item 1 odometry-pos) [
        ;print robot outside environment, stop model run
        set stop-run-flag true
        print (word current-sphero-pen-name " outside environment")
      ] [
        setxy (item 0 odometry-pos) (item 1 odometry-pos)
      ]
    ]

    set-current-plot "Speed / Heading scatter plot"
    set-current-plot-pen current-sphero-pen-name
    plotxy measured-speed abs-heading-diff
    file-open "scatter.log"
    file-print (word measured-speed "," abs-heading-diff)
    file-close

    ;; headings toggle
    ; only update headings if toggle is on
    if write-back-headings [
      set heading measured-heading
    ]

    ; plot target and measured speeds before target speeds are updated
    set-current-plot "Target speeds"
    set-current-plot-pen current-sphero-pen-name
    plot target-speed
    set-current-plot "Measured speeds"
    set-current-plot-pen current-sphero-pen-name
    plot measured-speed

    ; speed adaptations only after x ticks
    if ticks > 10 [
      ; we have 2 speed adaptation toggles, so we have 4 cases:

      ifelse cover-same-distances [
        ifelse move-at-same-measured-speed [
          ; case A: we only want to make adaptations if both distances and measured speeds indicate the same adaptation direction

          ifelse real-distance < target-distance [
            ifelse measured-speed < target-measured-speed [
              ;case A1: both distance and speed are too small, increase target speed!
              set target-speed (target-speed + speed-change-increment)
            ][
              ;case A2: distance is smaller but speed is bigger, do nothing for now
            ]
          ][
            ifelse measured-speed < target-measured-speed [
              ;case A3: distance is bigger but speed is smaller, do nothing for now
            ][
              ;case A4: both distance and speed are too big, decrease target speed!
              set target-speed (target-speed - speed-change-increment)
            ]
          ]


        ][
          ; case B: make distance-based adaptations only
          ifelse real-distance < target-distance [
            set target-speed (target-speed + speed-change-increment)
          ][
            set target-speed (target-speed - speed-change-increment)
          ]

        ]
      ][
        ifelse move-at-same-measured-speed [
          ; case C: make speed-based adaptations only
          ifelse measured-speed < target-measured-speed [
            set target-speed (target-speed + speed-change-increment)
          ][
            set target-speed (target-speed - speed-change-increment)
          ]

        ][
          ; case D: make no adaptations
          ; this at least updates the target-speed values if the start-percent-speed slider is used
          set target-speed start-percent-speed

        ]
      ]

      ; clamp target-speed to 1%-100%
      if target-speed < speed-change-increment [ set target-speed speed-change-increment]
      if target-speed > 100 [set target-speed 100]
    ]

  ]]

  ifelse (mean [real-distance] of spheros) < 0.1 [
    set target-distance 0.1 ; patch units
  ][
    set target-distance (mean [real-distance] of spheros)
  ]

  ; NNI
  if number-of-spheros > 1 [
    let current-nn-distance-mean mean [distance global-nearest-neighbor] of spheros
    set R current-nn-distance-mean / theoretical-nn-distance-mean
    set accumulated-R (accumulated-R + R)
  ]


end

to separate
  let nn-distance distance nearest-neighbor

  let turn-value 0

  ; allow full max-separate-turn value if this close together!
  ifelse nn-distance < 2.5 [
    set turn-value max-separate-turn
  ][
    ;otherwise inverse relation between nn-distance and how much the agent should turn away
    ; k and d may change during runtime so they need to be recalculated here every time
    let k (-1) * max-separate-turn / (min-separation - 2.5)
    let d max-separate-turn / (1 - 2.5 / min-separation)

    set turn-value k * nn-distance + d
  ]

  turn-away ([heading] of nearest-neighbor) turn-value
end







;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; HELPER FUNCTIONS BY URI WILENSKI ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


to flock
  find-flockmates
  ifelse any? flockmates [
    find-nearest-neighbor
    ifelse distance nearest-neighbor < min-separation [
      set shape "x"
      separate
    ][
      set shape "default"
      align
      cohere
    ]
  ][
    ; only apply randomness if no flockmates found
    wiggle wiggle-degrees
  ]
end

to find-flockmates ; origianlly by Uri Wilensky, adapted by Veronika Priesner
  set flockmates other spheros in-cone visibility visibility-cone ; Spheros only see flockmates in cone in front of them
end

to find-nearest-neighbor
  set nearest-neighbor min-one-of flockmates [distance myself]
end

to align
  turn-towards average-flockmate-heading max-align-turn
end

to-report average-flockmate-heading
  let x-component sum [dx] of flockmates
  let y-component sum [dy] of flockmates
  ifelse x-component = 0 and y-component = 0
    [ report heading ]
    [ report atan x-component y-component ]
end

to cohere
  turn-towards average-heading-towards-flockmates max-cohere-turn
end

to-report average-heading-towards-flockmates
  let x-component mean [sin (towards myself + 180)] of flockmates
  let y-component mean [cos (towards myself + 180)] of flockmates
  ifelse x-component = 0 and y-component = 0
    [ report heading ]
    [ report atan x-component y-component ]
end

to turn-towards [new-heading max-turn]
  turn-at-most (subtract-headings new-heading heading) max-turn
end

to turn-away [new-heading max-turn]
  turn-at-most (subtract-headings heading new-heading) max-turn
end

to turn-at-most [turn max-turn]
  ifelse abs turn > max-turn
    [ ifelse turn > 0
        [ rt max-turn ]
        [ lt max-turn ] ]
    [ rt turn ]
end

to wiggle [angle] ; ; originally from the Ant Lines model
  right random-float angle
  left random-float angle
end

; Copyright 1998 Uri Wilensky.
; See Info tab for full copyright and license.



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; HELPER FUNCTIONS BY VERONIKA PRIESNER ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

to write-config-file
  if file-exists? "config.txt" [
    file-delete "config.txt"
  ]

  file-open "config.txt"

  ; to catch if Python and NetLogo values are different
  file-print (word "number_of_spheros_in_netlogo = " (number-of-spheros))

  file-print (word "scale = " scale )

  ;write netlogo starting positions, is a 2d list
  file-type "first_netlogo_pos = ["
  foreach sort spheros [ s -> ask s [
    file-type (word "[" precision xcor 1 "," precision ycor 1 "]" ) ; mm precision
    if who + 1 - (count square-stamps) < number-of-spheros [
      file-type ", "
    ]
  ] ]
  file-print "]"


  ; Sphero color change commands
  foreach sort spheros [ s -> ask s [
    let r-rgb item 0 color
    let g-rgb item 1 color
    let b-rgb item 2 color

    file-print (word "sphero_array[" (who - tiles-per-env * tiles-per-env) "].set_rgb_led(" r-rgb ", " g-rgb ", " b-rgb ", 0, False)" )

  ] ]

  file-close

end

to clean-up
  clear-all

  ; delete old txt files
  file-close-all
  if file-exists? "commandsToRobots.txt" [
    file-delete "commandsToRobots.txt"
  ]
  if file-exists? "proceed.txt" [
    file-delete "proceed.txt"
  ]
end

to color-patches
  ask patches [

    ifelse (pxcor + pycor) mod 2 = 0 [
     set pcolor 0 ; black
   ][
     set pcolor 1 ; darkgrey
   ]

   ; green bottom orientation border
   if pycor = min-pycor [
     set pcolor green
   ]

    ; pink left border
   if pxcor = min-pxcor [
     set pcolor pink
   ]

    ; yellow top border
   if pycor = max-pycor [
     set pcolor yellow
   ]

    ; blue right border
   if pxcor = max-pxcor [
     set pcolor blue
   ]
  ]

end


to create-tiles
  create-square-stamps (tiles-per-env * tiles-per-env) [
     set color 3
     set shape "empty square"
     set size (patches-per-tile + 0.1)
     ]

   foreach sort square-stamps [ s -> ask s [
     let center-top-left (max-pxcor - floor(patches-per-tile / 2))
     let x (floor (who / tiles-per-env) * patches-per-tile - center-top-left)
     let y (who mod tiles-per-env * patches-per-tile - center-top-left)
     setxy x y
   ] ]
end

to set-up-spheros
    create-spheros number-of-spheros [
     set label who + 1 - (count square-stamps)
     set size 7.5 / scale ; diameter = 75 cm
     set color palette:scale-gradient palette:scheme-colors "Qualitative" "Set1" 8 (who - tiles-per-env * tiles-per-env + 1) 0 7 ; excludes red, the first color of Set1
     set flockmates no-turtles
     set pen-size 2
     set target-speed start-percent-speed
  ]

  place-spheros

  ;NNI: statistically significant R limits
  let area (world-width - 2) * (world-height - 2)
  set theoretical-nn-distance-mean 0.5 / sqrt(number-of-spheros / area)
  let standard-error sqrt(area * (4 - pi)/(4 * pi * number-of-spheros * number-of-spheros))
  set r-cluster-limit (-1.96 * standard-error + theoretical-nn-distance-mean) / theoretical-nn-distance-mean
  set r-regular-limit (1.96 * standard-error + theoretical-nn-distance-mean) / theoretical-nn-distance-mean

  foreach sort spheros [ s -> ask s [
    pen-down
  ]]
end

to set-up-plot-pens

  foreach sort spheros [ s -> ask s [
    let current-sphero-pen-name (word "Sphero " (who - 99))

    set-current-plot "Speed / Heading scatter plot"
    create-temporary-plot-pen current-sphero-pen-name
    set-plot-pen-mode 2
    set-plot-pen-color color

    set-current-plot "Target speeds"
    create-temporary-plot-pen current-sphero-pen-name
    set-plot-pen-color color

    set-current-plot "Measured speeds"
    create-temporary-plot-pen current-sphero-pen-name
    set-plot-pen-color color
  ]]
end









to-report add-vector [startpos vector]
  let x0 item 0 startpos
  let y0 item 1 startpos
  let xv item 0 vector
  let yv item 1 vector
  report list (x0 + xv) (y0 + yv)
end

to-report calculate-distance [pos-list-a pos-list-b]
  let diff-x (item 0 pos-list-b) - (item 0 pos-list-a)
  let diff-y (item 1 pos-list-b) - (item 1 pos-list-a)
  report sqrt(diff-x * diff-x + diff-y * diff-y)
end

to-report clamp-x-coord [x-coord]
  report (max(list min(list max-pxcor x-coord) min-pxcor))
end

to-report clamp-y-coord [y-coord]
  report (max(list min(list max-pycor y-coord) min-pycor))
end

to place-spheros
  ; 1-6 spheros place in equidistant triangle grid, slider determines how far apart they should be from each other
  let c starting-distance-between-spheros / scale
  let h c * sin(60)

  let right-down-vector list (c / 2) (-1 * h)
  let straight-left-vector list (-1 * c) (0)

  ; right flank first: go right and down 2x
  let pos-a [-0.5 -0.5]
  let pos-b add-vector pos-a right-down-vector
  let pos-c add-vector pos-b right-down-vector

  ; all positions left of the right flank positions
  let pos-d add-vector pos-b straight-left-vector
  let pos-e add-vector pos-c straight-left-vector
  let pos-f add-vector pos-e straight-left-vector

  foreach sort spheros [ s -> ask s [
     set heading 0
   ] ]


  ; 1 = A
  if number-of-spheros = 1 [
    ask sphero (count square-stamps) [
       setxy (item 0 pos-a) (item 1 pos-a)
     ]
   ]

  ; 2 = AE
  if number-of-spheros = 2 [
    ask sphero (count square-stamps) [
       setxy (item 0 pos-a) (item 1 pos-a)
     ]

    ask sphero (count square-stamps + 1) [
       setxy (item 0 pos-e) (item 1 pos-e)
     ]
   ]

  ; 3 = ABD
  if number-of-spheros = 3 [
    ask sphero (count square-stamps) [
       setxy (item 0 pos-a) (item 1 pos-a)
     ]

    ask sphero (count square-stamps + 1) [
       setxy (item 0 pos-b) (item 1 pos-b)
     ]

    ask sphero (count square-stamps + 2) [
       setxy (item 0 pos-d) (item 1 pos-d)
     ]
   ]

  ; 4 = ABDE
  if number-of-spheros = 4 [
    ask sphero (count square-stamps) [
       setxy (item 0 pos-a) (item 1 pos-a)
     ]

    ask sphero (count square-stamps + 1) [
       setxy (item 0 pos-b) (item 1 pos-b)
     ]

    ask sphero (count square-stamps + 2) [
       setxy (item 0 pos-d) (item 1 pos-d)
     ]

    ask sphero (count square-stamps + 3) [
       setxy (item 0 pos-e) (item 1 pos-e)
     ]
   ]

  ; 5 = ABDCF
  if number-of-spheros = 5 [
    ask sphero (count square-stamps) [
       setxy (item 0 pos-a) (item 1 pos-a)
     ]

    ask sphero (count square-stamps + 1) [
       setxy (item 0 pos-b) (item 1 pos-b)
     ]

    ask sphero (count square-stamps + 2) [
       setxy (item 0 pos-d) (item 1 pos-d)
     ]

    ask sphero (count square-stamps + 3) [
       setxy (item 0 pos-c) (item 1 pos-c)
     ]

    ask sphero (count square-stamps + 4) [
       setxy (item 0 pos-f) (item 1 pos-f)
     ]
   ]

  ; 6 = ABDCEF
  if number-of-spheros = 6 [
    ask sphero (count square-stamps) [
       setxy (item 0 pos-a) (item 1 pos-a)
     ]

    ask sphero (count square-stamps + 1) [
       setxy (item 0 pos-b) (item 1 pos-b)
     ]

    ask sphero (count square-stamps + 2) [
       setxy (item 0 pos-d) (item 1 pos-d)
     ]

    ask sphero (count square-stamps + 3) [
       setxy (item 0 pos-c) (item 1 pos-c)
     ]

    ask sphero (count square-stamps + 4) [
       setxy (item 0 pos-e) (item 1 pos-e)
     ]

    ask sphero (count square-stamps + 5) [
       setxy (item 0 pos-f) (item 1 pos-f)
     ]
   ]

end

to avoid-borders
  let heading-towards-center towardsxy 0 0
  turn-towards heading-towards-center max-center-turn
end

to send-commands
  file-open "commandsToRobots.txt"

  ; target headings are written at every tick to python script
  ; send execute command to overwrite Sphero headings array
  ; sphero_target_headings = [h1, h2, h3, h4, h5]
  file-type "sphero_target_headings = ["
  foreach sort spheros [ s -> ask s [
    file-type precision heading 2
    if who + 1 - (count square-stamps) < number-of-spheros [
      file-type ", "
    ]
  ] ]
  file-print "]"

  ;send sphero target speeds to python script
  file-type "sphero_target_speeds = ["
  foreach sort spheros [ s -> ask s [
    file-type target-speed
    if who + 1 - (count square-stamps) < number-of-spheros [
      file-type ", "
    ]
  ] ]
  file-print "]"

  file-close
end


; Copyright 2018 Veronika Priesner.
@#$#@#$#@
GRAPHICS-WINDOW
252
10
780
539
-1
-1
10.0
1
8
1
1
1
0
0
0
1
-26
25
-26
25
0
0
1
ticks
60.0

BUTTON
797
461
1042
497
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
796
506
916
541
NIL
start
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
0

SLIDER
16
466
245
499
max-align-turn
max-align-turn
0.0
12
6.0
1
1
°
HORIZONTAL

SLIDER
17
504
244
537
max-cohere-turn
max-cohere-turn
0.0
12
2.0
1
1
°
HORIZONTAL

SLIDER
15
427
244
460
max-separate-turn
max-separate-turn
0.0
12
2.0
1
1
°
HORIZONTAL

SLIDER
14
351
243
384
visibility
visibility
10
25
15.0
1
1
patches
HORIZONTAL

SLIDER
15
388
249
421
min-separation
min-separation
1.5
10
5.0
0.5
1
patches
HORIZONTAL

SLIDER
17
139
246
172
max-center-turn
max-center-turn
0
20
16.0
1
1
°
HORIZONTAL

SLIDER
12
47
240
80
number-of-spheros
number-of-spheros
1
6
4.0
1
1
NIL
HORIZONTAL

SLIDER
17
212
246
245
wiggle-degrees
wiggle-degrees
0
20
10.0
1
1
°
HORIZONTAL

SLIDER
796
53
1039
86
tiles-per-env
tiles-per-env
4
16
10.0
2
1
tiles per side
HORIZONTAL

SLIDER
795
10
1039
43
patches-per-tile
patches-per-tile
1
9
5.0
2
1
patches  = 1 tile
HORIZONTAL

BUTTON
927
509
1043
542
stop
set stop-run-flag true
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
0

MONITOR
796
239
915
284
Current world min
min-pxcor
17
1
11

MONITOR
924
239
1042
284
Set world mins to:
-1 * (patches-per-tile * tiles-per-env + 2) / 2
17
1
11

MONITOR
925
294
1044
339
Set world maxs to:
(patches-per-tile * tiles-per-env + 2) / 2 - 1
17
1
11

MONITOR
796
295
915
340
Current world max
max-pxcor
17
1
11

MONITOR
924
185
1041
230
Set patch size to:
(word (round (500 / (patches-per-tile * tiles-per-env + 2))) \" pixels\")
17
1
11

MONITOR
795
184
914
229
Current patch size
(word patch-size \" pixels\")
17
1
11

MONITOR
798
406
1042
451
Calculated real-world dimensions
(word (scale * 2 * max-pxcor) \"x\" (scale * 2 * max-pxcor) \" cm\")
17
1
11

MONITOR
796
350
1042
395
Number of patches
(word (world-width - 2) \"x\" (world-height - 2))
17
1
11

SLIDER
795
96
1039
129
scale
scale
3
7
5.0
1
1
cm = 1 patch-width
HORIZONTAL

SLIDER
794
139
1041
172
starting-distance-between-spheros
starting-distance-between-spheros
10
50
30.0
5
1
cm
HORIZONTAL

SLIDER
14
313
243
346
visibility-cone
visibility-cone
0
360
340.0
10
1
°
HORIZONTAL

SLIDER
18
176
247
209
border-avoidance-distance
border-avoidance-distance
1
25
10.0
1
1
patches
HORIZONTAL

PLOT
10
551
445
741
 Mean distance between target and received positions
Ticks
Distance (cm)
0.0
25.0
0.0
1.0
true
false
"" ""
PENS
"current mean distance" 1.0 0 -16777216 true "" "if ticks > 0 [\n  plot mean [target-to-odometry-distance] of spheros\n]"

SLIDER
16
103
246
136
start-percent-speed
start-percent-speed
5
25
15.0
1
1
%
HORIZONTAL

SWITCH
13
10
239
43
mock-robots
mock-robots
1
1
-1000

PLOT
955
553
1414
737
Target speeds
Ticks
% of max. speed
0.0
25.0
0.0
1.0
true
false
"" ""
PENS
"0 %" 1.0 0 -7500403 true "" "plot 0"
"mean target speed" 1.0 0 -16777216 true "" "if ticks > 0 [\n  plot mean [target-speed] of spheros\n]"

SWITCH
1238
14
1422
47
write-back-headings
write-back-headings
1
1
-1000

SWITCH
1057
13
1239
46
write-back-positions
write-back-positions
0
1
-1000

MONITOR
1054
497
1243
542
Mean time passed per tick
(word (precision (model-timer / ticks) 3) \" s\")
17
1
11

BUTTON
1252
500
1416
542
clear paths
clear-drawing
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

PLOT
1055
228
1415
486
Nearest Neighbor Index (NNI)
NIL
NIL
0.0
25.0
0.0
2.149
true
true
"" ""
PENS
"Current R" 1.0 0 -16777216 true "" "ifelse R < r-cluster-limit [\n  set-plot-pen-color green\n][\n  ifelse R < r-regular-limit [\n    set-plot-pen-color red\n  ][\n    set-plot-pen-color blue\n  ]\n]\n\nif number-of-spheros > 1 and ticks > 0 [\n  plot R\n]"
"Regularity limit" 1.0 0 -8020277 true "" "if number-of-spheros > 1 [\n  plot r-regular-limit\n]"
"Clustering limit" 1.0 0 -6565750 true "" "if number-of-spheros > 1 [\n  plot r-cluster-limit\n]"

MONITOR
44
573
298
618
Current mean distance between positions
(word precision (mean [target-to-odometry-distance] of spheros) 1 \" cm\")
17
1
11

PLOT
954
750
1416
922
Measured speeds
Ticks
Meas. speed (cm/s)
0.0
25.0
0.0
1.0
true
false
"" ""
PENS
"0 cm/s" 1.0 0 -7500403 true "" "plot 0"
"current mean m. speed" 1.0 0 -16777216 true "" "if ticks > 0 [\n  plot mean [measured-speed] of spheros\n]"

MONITOR
1090
858
1290
903
Current mean measured speed
(word precision mean [measured-speed] of spheros 1 \" cm/s\")
17
1
11

MONITOR
1093
674
1283
719
Current mean target speed
(word precision mean [target-speed] of spheros 1 \" %\")
17
1
11

SLIDER
1057
137
1331
170
target-measured-speed
target-measured-speed
1
50
23.0
1
1
cm / s
HORIZONTAL

SWITCH
1058
60
1330
93
cover-same-distances
cover-same-distances
1
1
-1000

SWITCH
1057
105
1331
138
move-at-same-measured-speed
move-at-same-measured-speed
1
1
-1000

PLOT
10
754
442
926
Mean covered distance per tick
Ticks
Covered distance (cm)
0.0
25.0
0.0
1.0
true
false
"" ""
PENS
"mean real distance in cm" 1.0 0 -16777216 true "" "if ticks > 0 [\n  plot (mean [real-distance] of spheros) * scale\n]"

MONITOR
44
777
264
822
Current mean distance covered
(word precision ((mean [real-distance] of spheros) * scale) 1 \" cm\")
17
1
11

SLIDER
1058
182
1332
215
speed-change-increment
speed-change-increment
0.25
1
0.25
0.25
1
%
HORIZONTAL

PLOT
460
553
940
926
Speed / Heading scatter plot
Measured speed (cm/s)
Abs. head. diff. (°)
0.0
50.0
0.0
180.0
true
false
"" ""
PENS

MONITOR
1279
426
1413
471
Mean R over time
precision (accumulated-R / ticks) 3
17
1
11

MONITOR
1279
382
1414
427
Clustering limit
precision r-cluster-limit 3
17
1
11

MONITOR
724
576
917
621
Mean measured speed over time
(word precision ((mean [accumulated-measured-speeds] of spheros) / ticks) 1 \" cm/s\")
17
1
11

@#$#@#$#@
## TODO write documentation into model


## WHAT IS IT?

This model is an attempt to mimic the flocking of birds.  (The resulting motion also resembles schools of fish.)  The flocks that appear in this model are not created or led in any way by special leader birds.  Rather, each bird is following exactly the same set of rules, from which flocks emerge.

## HOW IT WORKS

The birds follow three rules: "alignment", "separation", and "cohesion".

"Alignment" means that a bird tends to turn so that it is moving in the same direction that nearby birds are moving.

"Separation" means that a bird will turn to avoid another bird which gets too close.

"Cohesion" means that a bird will move towards other nearby birds (unless another bird is too close).

When two birds are too close, the "separation" rule overrides the other two, which are deactivated until the minimum separation is achieved.

The three rules affect only the bird's heading.  Each bird always moves forward at the same constant speed.

## HOW TO USE IT

First, determine the number of birds you want in the simulation and set the POPULATION slider to that value.  Press SETUP to create the birds, and press GO to have them start flying around.

The default settings for the sliders will produce reasonably good flocking behavior.  However, you can play with them to get variations:

Three TURN-ANGLE sliders control the maximum angle a bird can turn as a result of each rule.

VISION is the distance that each bird can see 360 degrees around it.

## THINGS TO NOTICE

Central to the model is the observation that flocks form without a leader.

There are no random numbers used in this model, except to position the birds initially.  The fluid, lifelike behavior of the birds is produced entirely by deterministic rules.

Also, notice that each flock is dynamic.  A flock, once together, is not guaranteed to keep all of its members.  Why do you think this is?

After running the model for a while, all of the birds have approximately the same heading.  Why?

Sometimes a bird breaks away from its flock.  How does this happen?  You may need to slow down the model or run it step by step in order to observe this phenomenon.

## THINGS TO TRY

Play with the sliders to see if you can get tighter flocks, looser flocks, fewer flocks, more flocks, more or less splitting and joining of flocks, more or less rearranging of birds within flocks, etc.

You can turn off a rule entirely by setting that rule's angle slider to zero.  Is one rule by itself enough to produce at least some flocking?  What about two rules?  What's missing from the resulting behavior when you leave out each rule?

Will running the model for a long time produce a static flock?  Or will the birds never settle down to an unchanging formation?  Remember, there are no random numbers used in this model.

## EXTENDING THE MODEL

Currently the birds can "see" all around them.  What happens if birds can only see in front of them?  The `in-cone` primitive can be used for this.

Is there some way to get V-shaped flocks, like migrating geese?

What happens if you put walls around the edges of the world that the birds can't fly into?

Can you get the birds to fly around obstacles in the middle of the world?

What would happen if you gave the birds different velocities?  For example, you could make birds that are not near other birds fly faster to catch up to the flock.  Or, you could simulate the diminished air resistance that birds experience when flying together by making them fly faster when in a group.

Are there other interesting ways you can make the birds different from each other?  There could be random variation in the population, or you could have distinct "species" of bird.

## NETLOGO FEATURES

Notice the need for the `subtract-headings` primitive and special procedure for averaging groups of headings.  Just subtracting the numbers, or averaging the numbers, doesn't give you the results you'd expect, because of the discontinuity where headings wrap back to 0 once they reach 360.

## RELATED MODELS

* Moths
* Flocking Vee Formation
* Flocking - Alternative Visualizations

## CREDITS AND REFERENCES

This model is inspired by the Boids simulation invented by Craig Reynolds.  The algorithm we use here is roughly similar to the original Boids algorithm, but it is not the same.  The exact details of the algorithm tend not to matter very much -- as long as you have alignment, separation, and cohesion, you will usually get flocking behavior resembling that produced by Reynolds' original model.  Information on Boids is available at http://www.red3d.com/cwr/boids/.

## HOW TO CITE

If you mention this model or the NetLogo software in a publication, we ask that you include the citations below.

For the model itself:

* Wilensky, U. (1998).  NetLogo Flocking model.  http://ccl.northwestern.edu/netlogo/models/Flocking.  Center for Connected Learning and Computer-Based Modeling, Northwestern University, Evanston, IL.

Please cite the NetLogo software as:

* Wilensky, U. (1999). NetLogo. http://ccl.northwestern.edu/netlogo/. Center for Connected Learning and Computer-Based Modeling, Northwestern University, Evanston, IL.

## COPYRIGHT AND LICENSE

Copyright 1998 Uri Wilensky.

![CC BY-NC-SA 3.0](http://ccl.northwestern.edu/images/creativecommons/byncsa.png)

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 License.  To view a copy of this license, visit https://creativecommons.org/licenses/by-nc-sa/3.0/ or send a letter to Creative Commons, 559 Nathan Abbott Way, Stanford, California 94305, USA.

Commercial licenses are also available. To inquire about commercial licenses, please contact Uri Wilensky at uri@northwestern.edu.

This model was created as part of the project: CONNECTED MATHEMATICS: MAKING SENSE OF COMPLEX PHENOMENA THROUGH BUILDING OBJECT-BASED PARALLEL MODELS (OBPML).  The project gratefully acknowledges the support of the National Science Foundation (Applications of Advanced Technologies Program) -- grant numbers RED #9552950 and REC #9632612.

This model was converted to NetLogo as part of the projects: PARTICIPATORY SIMULATIONS: NETWORK-BASED DESIGN FOR SYSTEMS LEARNING IN CLASSROOMS and/or INTEGRATED SIMULATION AND MODELING ENVIRONMENT. The project gratefully acknowledges the support of the National Science Foundation (REPP & ROLE programs) -- grant numbers REC #9814682 and REC-0126227. Converted from StarLogoT to NetLogo, 2002.

<!-- 1998 2002 -->
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

empty square
false
0
Rectangle -7500403 false true 0 0 300 300

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

orbit
true
0
Circle -7500403 false true 0 0 300

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

visibility
true
0
Polygon -7500403 false true 300 150 0 150 150 0

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.0.2
@#$#@#$#@
set population 200
setup
repeat 200 [ go ]
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
