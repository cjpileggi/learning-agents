;;--------------------------------------------------------------------------------
;;
;; learningTurtles.nlogo
;;
;; A simulation of a robot moving around an environment to find a goal using either
;; random movements, heuristics or reinforced learning algorithms
;;
;; Christopher Pileggi
;; October 2012
;;
;;


;;--------------------------------------------------------------------------------
;;
;; We have one breed and a few global variables

breed [robots robot]
globals [g-xcor g-ycor r-xcor r-ycor done goal-found] ;; goal-found shows whether or not
                                                      ;; a reinforcement learning was successful
;; For every node
patches-own[h-val] ;; the heuristic value of the node


;;--------------------------------------------------------------------------------
;;
;; Code to set up the environment

;; setup
;;
;; To setup we get rid of any existing robots, color the background, pick obstacle
;; locations and a goal, and then put the robot down in a random position

to setup
  clear-all  ;; clear everything
  ask turtles [die]
  reset-ticks
  clear-output  ;; clear output box
  set goal-found -1  ;; indicates the reinforcement learning has not happened
  ask patches[
      set h-val 0  ;; set all patch values to 0
      set plabel "";; clear labels
  ]
  ask patches [set pcolor green] ;; Setup environment with all green patches
  repeat obstacles [ask one-of patches [set pcolor black]] ;; Add obstacles (black patches) at random locations
  ask one-of patches [ ;; set the goal patch
    set pcolor red
    set g-xcor pxcor
    set g-ycor pycor
  ]

  ;; Position Robot

  ;; setup the robot breed
  set-default-shape robots "robot"
  create-robots 1 [set color black]

  ;; place the robot
  ;; remember the initial location of the robot to allow us to reset

  set r-xcor free-x-cor
  set r-ycor free-y-cor

  ask robots [set xcor r-xcor]
  ask robots [set ycor r-ycor]
  ask robots [set heading 0]

  ;; make sure there isn't an obstacle at the robot's initial position
  ask patches [if ((pxcor = r-xcor) and (pycor = r-ycor)) [set pcolor green]]
  set done false
end

;; free-x-cor free-y-cor
;;
;; These are supposed to generate random locations that aren't obstacles. For
;; now they just report 0 or a random location (depending on the value of
;; robot-position, and we adjust the obstacles to avoid them.

to-report free-x-cor
  if-else robot-position = "center"[report 0] [report random-pxcor]
end

to-report free-y-cor
  if-else robot-position = "center"[report 0] [report random-pycor]
end

;; reset
;;
;; Reset allows you to return the robot to its initial position to run more
;; than once in the same environment.

to reset
  ask robots [set xcor r-xcor set ycor r-ycor]
  ask patches[set h-val 0 set plabel ""]
  reset-ticks
  clear-output
  set done false
  repaint-space
  set goal-found -1
end

;; repaint-space
;;
;; In case we have been looking at the search,

to repaint-space
  ask patches [if (pcolor != black) and (pcolor != red)[set pcolor green]]
end

;;--------------------------------------------------------------------------------
;;
;;

;; go
;;
;; The top level loop. Make the robot take a step forward, tick, and stop if the
;; robot is at the goal (which stops the clock).

to go
  run-robot
  tick
  ask robots [at-goal-test]
  if done[stop]
end

;; run-robot
;;
;; The high level call to make the robots move --- as long as you aren't at the
;; goal, make a move.

to run-robot
  ask robots [if-else not at-goal [robot-move][set color green]]
end

;; robot-move
;;
;; The place where we plug in different ways to move the robot. The basic
;; mechanism is just to make a random move.
;; Can choose between random movement, heuristic movement, or reinforcement learning

to robot-move
  if movement = "random" [robot-random-move]
  if movement = "heuristic"[robot-h-move]
  if movement = "reinforcement-learning"[robot-learn-move]
end

;; robot-random-move
;;
;; Pick a direction, randomly and with equal probability

to robot-random-move
  let direction random 4

  ;;show direction
  if direction = 0 [move "north"]
  if direction = 1 [move "south"]
  if direction = 2 [move "east"]
  if direction = 3 [move "west"]
end

;; robot-h-move
;;
;;
;; Main function for doing a heuristic search. All the work is done by
;; assing-surrounding-nodes. Passes true to assign-surrounding-nodes to indicate
;; it has do a heuristic search

to robot-h-move
  assign-surrounding-nodes true
end

;; assign-node-heur
;;
;; assigns a node a value based on a heuristic. The heuristic here is: the smaller
;; the distance between the current node and the goal, the greater the heuristic
;; value. This function calls the distance function and gives that value to
;; the node. It also sees if this is this value is best value of the set from the
;; assign-surrounding-nodes function and returns that value.

to-report assign-node-heur [curr-node best-in-set]
  ;; temp x and y coodinates for the current node
  let h-xcord [pxcor] of curr-node
  let h-ycord [pycor] of curr-node

  ;; find the distance between the current node and goal node
  let dist line-dist g-xcor h-xcord g-ycor h-ycord

  ;; assign current node its heuristic value
  ask curr-node[set h-val dist]

  ;; if this is the best in the set so far, assign best-in-set the value
  if (best-in-set >= dist)[set best-in-set dist]

  report best-in-set
end

;; move
;;
;; You can move in a direction as long as there is no obstacle.

to move [direction]
  ask robots [if not obstacle direction patch-here[step direction]]
end

;; obstacle
;;
;; We spot an obstacle by identifying if there is a black patch in the
;; requisite direction, or if we are against the edge of the world.

to-report obstacle[direction my-patch]

  let a-color green

  ;; First look at the color of the patch in the right direction, taking care of
  ;; the edges of the field, and repeat for all four directions.

  if direction = "north" [
    if-else patch ([pxcor] of my-patch) (([pycor] of my-patch) + 1) = nobody [set a-color black]
    [set a-color [pcolor] of patch ([pxcor] of my-patch) (([pycor] of my-patch) + 1)]
  ]
  if direction = "south" [
    if-else patch ([pxcor] of my-patch) (([pycor] of my-patch) - 1) = nobody [set a-color black]
    [set a-color [pcolor] of patch ([pxcor] of my-patch) (([pycor] of my-patch) - 1)]
  ]
  if direction = "east" [
    if-else patch (([pxcor] of my-patch) + 1) ([pycor] of my-patch) = nobody [set a-color black]
    [set a-color [pcolor] of patch (([pxcor] of my-patch) + 1) ([pycor] of my-patch)]
  ]
  if direction = "west" [
    if-else patch (([pxcor] of my-patch) - 1) ([pycor] of my-patch) = nobody [set a-color black]
    [set a-color [pcolor] of patch (([pxcor] of my-patch) - 1) ([pycor] of my-patch)]
  ]

  ;; Then report based on the color
  if-else a-color = black [report true][report false]
end

;; step
;;
;; Move in the requisite direction, making each step exactly one patch.

to step [direction]
  if direction = "north" [set ycor [pycor] of patch-at 0 1]
  if direction = "south" [set ycor [pycor] of patch-at 0 -1]
  if direction = "east" [set xcor [pxcor] of patch-at 1 0]
  if direction = "west" [set xcor [pxcor] of patch-at -1 0]
end

;; at-goal-test
;;
;; robot signals it has got to the goal so that we can stop the simulation

to at-goal-test
  if is-goal patch-here [set done true]
end

;; at-goal
;;
;; robot knows it has reached the goal

to-report at-goal
  if-else is-goal patch-here [report true][report false]
end

;; is-goal
;;

to-report is-goal [my-patch]
  let x-cor [pxcor] of my-patch
  let y-cor [pycor] of my-patch
  if-else ((x-cor = g-xcor) and  (y-cor = g-ycor))[report true][report false]
end

;;---------------------------------------------------------------------------
;;

;; robot-learn-move
;;
;; Main function for doing reinforcement learning. It usfes the value-iteration
;; function to assign node values. Then it works according to whatever value-iteration
;; sets goal-found to be. It either indicates a failure to find the goal in the robot's
;; path or moves the robot to the goal using assign-surrounding-nodes.

to robot-learn-move

  ;; if a value iteration has not been done, do it
  if goal-found = -1[
   reset-timer ;; let the timer start
   set goal-found value-iteration
  ]

  ;; if failure to have the goal in the robots path
  if goal-found = 0 [
    output-print "failed"
    set done true
  ]

  ;; if goal is in robots path, call assign-surrounding-nodes. Sets false to indicate
  ;; reinforcement learning
  if goal-found = 1 [assign-surrounding-nodes false]
end

;; value-iteration
;;
;; Function used for reinforcement learning. It looks at every node in the world and
;; assigns them a value acccording to the Bellman equation, which is done using the
;; assign-node-bellman function. The obstacles are given a value of -1 and the goal is
;; given a value of 0. Nodes that are blocked off by obstacles are also given a a value
;; of -1. At the end of an iteration, it sees if the values are stable. If they are not,
;; this function calls itself to reevaluate the world again.
;; If they are stable, it reports either a 0 or 1.
;; 0 means the goal cannot be reached by the robot, and 1 meaning it can.

to-report value-iteration

  let stable 0 ;; determine swhether or not the values are stable
  let goal-status -1
  let all-y max-pycor

  while [all-y != (min-pycor - 1)][  ;; nested while loop: used to search every node in the
                                     ;; world, looking from left to right and top to bottom
    let all-x min-pxcor

    while [all-x != (max-pxcor + 1)][
      let curr-node (patch all-x all-y) ;; node we are currently evaluating based on loop

      ;; first, analyze the node to see if it is an obstacle, goal, or is blocked
      if-else is-goal curr-node[  ;; goal node always has a 0 value
        ask curr-node[
          set h-val 0
          if show-node-values[set plabel h-val]
        ]  ;; show value if chosen
      ]
      [
        ;; nodes that are obstacles
        ;; or blocked are set to -1.
        ;; Sets node to cutoff if its value is too big

        ;; Estimate the number of possible patches the agent can traverse
        ;; Add the maximum x and y values of the environment and one tenth of the
        ;; obstacles to account for any contorted paths
        let mx world-width + world-height + obstacles / 10

        ask curr-node[
          if (pcolor = black) or (h-val >= mx) [set h-val -1]
          if-else (h-val = -1)[set plabel ""][if show-node-values[set plabel h-val]]  ;; show value if chosen
        ]


        ;; if the node is not an obstacle or goal, assign the node its value.
        ;; Looks at the value of the node before and after and
        ;; see if the values are stable
        let val 0
        ask curr-node[set val h-val]
        if val != -1 [
          if not is-goal curr-node[
            let before 0
            ask curr-node[set before h-val]  ;; value before this assignment
            let s-nodes (find-surrounding-nodes curr-node) ;; find surronding nodes

            assign-node-bellman curr-node s-nodes ;; assign current node based on surrounding nodes

            let after 0
            ask curr-node[set after h-val] ;; value after the assignment
            set-node-color curr-node

            ;; if values before and after assignment are not the same, it is unstable
            if before != after [set stable (stable + 1)]
          ]
        ]
      ]

      set-node-color curr-node ;; color the node according to value
      set all-x (all-x + 1)
    ] ;; end inner loop
    set all-y (all-y - 1)
  ] ;; end outer loop

  ;; end of evaluation

  ;; see if the node values are stable
  ;; if stable = 0 report the goal status
  ;; if the robot's initial node has a value of -1, that means
  ;; it has been blocked off from the goal
  ;; otherwise, print the time and set success

  if stable = 0[
    ask patch-here[
      if-else (h-val = -1)
        [set goal-status 0]
        [
          ;; stop the timer and display the amount of time it took to find the correct values of the nodes.
          output-type "Success! It took " output-type timer output-print " seconds to find a stable state."
          set goal-status 1
        ]
    ]
    report goal-status
  ]

  report value-iteration ;; one or more nodes are not stable
                         ;; repeat function again
end

;; assign-node-bellman
;;
;; Uses the variation of the Bellman equaion to assign the value of a node. It looks at
;; the node's children and looks at which has the smallest value.
;; The current node is assigned that value. Added to that value
;; is the pathcost to get to that node, which in this case is 1

to assign-node-bellman[curr-node surr-nodes]

  let best 999 ;; best value in set

  foreach surr-nodes[
    let temp 0
    ask ?[set temp h-val]

    if (temp <= best)[set best temp] ;; if the value of the current surrounding
  ]                                  ;; node is the lowest so far, assign thay value to best

  ;; whatever the best value is, the current node is given that value.
  ;; Added to that value is the path cost between the current node and
  ;; the selected child node
  ask curr-node[set h-val (best + 1)]
end

;; assign-surrounding-nodes
;;
;; looks at the the nodes that currently surround the robot and chooses which is
;; the best for the robot to move to based on its value. This function is called
;; for both the Heuristic Movement and the Reiforcment Learning. This depends on the
;; value given to heur-or-learn (true = heuristic movement; false = reinforcement learning).
;;
;; If true, the surronding nodes are assigned heuristic values using the assign-node-heur
;; function.
;;
;; If false,it just finds the value of the best node, since their values have
;; already been assigned by value-iteration.
;;
;; select-direction-from-best is called to determine the best direction for the robot to move in

to assign-surrounding-nodes [heur-or-learn]

  ;; temp values for surronding nodes
  let n-val 0
  let s-val 0
  let e-val 0
  let w-val 0

  ;; nodes surrounding the node the robot is on
  let n-node patch-at 0 1
  let s-node patch-at 0 -1
  let e-node patch-at 1 0
  let w-node patch-at -1 0

  ;; used to see what the best value of the surounding nodes is
  let best-in-set 999

  ;; For the top node:
  if n-node != nobody[  ;; looks to see if there is actually a node or nothing

    if-else heur-or-learn = true[ ;; if using heuristic movement:

      if-else (obstacle "north" patch-here)[  ;; if node is an obstacle
        ask n-node[set h-val -1]
      ]
      [
        set best-in-set (assign-node-heur n-node best-in-set)  ;; call assign-node-heur
        ask n-node[ set n-val h-val] ;; set temporary north value
      ]
    ]
    [  ;; else, if reinforcement learning was done:
      ask n-node[ set n-val h-val]
      if (n-val <= best-in-set and (n-val != -1)) [set best-in-set n-val] ;; set best-in-set
    ]
  ]

  ;; For the bottom node:
  if s-node != nobody[
    if-else heur-or-learn = true [
      if-else (obstacle "south" patch-here)[
        ask s-node[set h-val -1]
      ]
      [
        set best-in-set (assign-node-heur s-node best-in-set)
        ask s-node[ set s-val h-val]
      ]
    ]
    [
      ask s-node[ set s-val h-val]
      if (s-val <= best-in-set and (s-val != -1)) [set best-in-set s-val]
    ]
  ]

  ;; For the right node:
  if e-node != nobody[
    if-else heur-or-learn = true[
      if-else (obstacle "east" patch-here)[
        ask e-node[set h-val -1]
      ]
      [
        set best-in-set (assign-node-heur e-node best-in-set)
        ask e-node[ set e-val h-val]
      ]
    ]
    [
      ask e-node[ set e-val h-val]
      if (e-val <= best-in-set and (e-val != -1)) [set best-in-set e-val]
    ]
  ]

  ;; For the left node:
  if w-node != nobody[
    if-else heur-or-learn = true[
      if-else (obstacle "west" patch-here)[
        ask w-node[set h-val -1]
      ]
      [
        set best-in-set (assign-node-heur w-node best-in-set)
        ask w-node[ set w-val h-val]
      ]
    ]
    [
      ask w-node[ set w-val h-val]
      if (w-val <= best-in-set and (w-val != -1)) [set best-in-set w-val]
    ]
  ]

  ;; show path if chosen to
  if (show-path and not is-goal patch-here)[ ask patch-here [set pcolor magenta]]

  ;; set the direction the robot should move in based on best-in-set
  let direction (select-direction-from-best n-val s-val e-val w-val best-in-set)

  ;; move the robot in the selected direction
  move direction

  ;; show node values if chosen to
  if (show-node-values)[ask patch-here [set plabel precision h-val 1]]
end

;; select-direction-from-best
;;
;; called from assign-surrounding nodes. This compares the values of the surrounding
;; nodes, and sets the direction according to which node has the best value.
;; If 2 or more nodes have the best value, they are selected at random.
;; It returns the direction

to-report select-direction-from-best [n-val s-val e-val w-val best-in-set]

  let direction ""
  let rand random 4  ;; for choosing a direction

  ;; when rand is chosen, a corresponding if statement is called.
  ;; if the direction chosen has a value thats best in the set, its sets
  ;; that direction. If it is not, it sets a NULL

  if rand = 0[if-else n-val = best-in-set [set direction "north"] [set direction "NULL"]]
  if rand = 1[if-else s-val = best-in-set [set direction "south"] [set direction "NULL"]]
  if rand = 2[if-else e-val = best-in-set [set direction "east"] [set direction "NULL"]]
  if rand = 3[if-else w-val = best-in-set [set direction "west"] [set direction "NULL"]]

  ;; if direction was null, call this function again
  ;; else, return the direction
  if-else direction = "NULL"[
    report (select-direction-from-best n-val s-val e-val w-val best-in-set)
  ]
  [
    report direction
  ]
end

;; set-color-node
;;
;; sets the color of the nodes according to their heuristic values. The closer the
;; value is to 0, the lighter green the node is. The farther the value is from 0,
;; the darker green it is.

to set-node-color[curr-node]

  ask curr-node[
    if (h-val = 1)[set pcolor [ 150 255 150 ]]
    if (h-val = 2)[set pcolor [ 120 255 120 ]]
    if (h-val = 3)[set pcolor [ 90 255 90 ]]
    if (h-val = 4)[set pcolor [ 60 255 60 ]]
    if (h-val = 5)[set pcolor [ 30 255 30 ]]
    if (h-val = 6)[set pcolor [ 0 255 0 ]]
    if (h-val = 7)[set pcolor [ 0 245 0 ]]
    if (h-val = 8)[set pcolor [ 0 235 0 ]]
    if (h-val = 9)[set pcolor [ 0 225 0 ]]
    if (h-val = 10)[set pcolor [ 0 215 0 ]]
    if (h-val = 11)[set pcolor [ 0 205 0 ]]
    if (h-val = 12)[set pcolor [ 0 195 0]]
    if (h-val = 13)[set pcolor [ 0 185 0 ]]
    if (h-val = 14)[set pcolor [ 0 175 0 ]]
    if (h-val = 15)[set pcolor [ 0 165 0 ]]
    if (h-val = 16)[set pcolor [ 0 155 0 ]]
    if (h-val = 17)[set pcolor [ 0 145 0 ]]
    if (h-val = 18)[set pcolor [ 0 135 0 ]]
    if (h-val = 19)[set pcolor [ 0 125 0 ]]
    if (h-val = 20)[set pcolor [ 0 115 0 ]]
    if (h-val = 21)[set pcolor [ 0 105 0 ]]
    if (h-val = 22)[set pcolor [ 0 95 0 ]]
    if (h-val = 23)[set pcolor [ 0 85 0 ]]
    if (h-val = 24)[set pcolor [ 0 75 0 ]]
    if (h-val = 25)[set pcolor [ 0 65 0 ]]
    if (h-val > 25)[set pcolor [ 0 55 0 ]]

    ;; if a node is blocked
    if (h-val = -1)[if (pcolor != black)[set pcolor yellow]]
  ]
end

;; find-surrounding-nodes
;;
;; given a patch and a list of patches, finds the patches north, south, east
;; and west of the given patch that are in the field and not obstacles, and
;; returns them

to-report find-surrounding-nodes [my-patch]
  let node-list []
  let new-patch nobody
  if not obstacle "north" my-patch [
    ask my-patch [set new-patch patch-at 0 1]
    if new-patch != nobody [set node-list lput new-patch node-list]
  ]
  set new-patch 0
  if not obstacle "south" my-patch [
    ask my-patch [set new-patch patch-at 0 -1]
    if new-patch != nobody [set node-list lput new-patch node-list]
  ]
  set new-patch 0
  if not obstacle "east" my-patch [
    ask my-patch [set new-patch patch-at 1 0]
    if new-patch != nobody [set node-list lput new-patch node-list]
  ]
  set new-patch 0
  if not obstacle "west" my-patch [
    ask my-patch [set new-patch patch-at -1 0]
    if new-patch != nobody [set node-list lput new-patch node-list]
  ]
  report node-list
end


;; line-dist
;;
;; Line Distance Formula
;; calculates the distance between 2 nodes
to-report line-dist [x1 x2 y1 y2]
  let dist sqrt(((x1 - x2)*(x1 - x2)) + ((y1 - y2)*(y1 - y2)))
  report dist
end
@#$#@#$#@
GRAPHICS-WINDOW
256
75
673
469
18
16
11.0
1
10
1
1
1
0
0
0
1
-18
18
-16
16
0
0
1
ticks
30.0

BUTTON
28
39
94
72
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

SLIDER
28
80
223
113
obstacles
obstacles
0
1000
594
1
1
NIL
HORIZONTAL

BUTTON
27
225
90
258
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

MONITOR
19
269
76
314
NIL
ticks
0
1
11

BUTTON
102
225
165
258
NIL
reset
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

CHOOSER
28
118
222
163
movement
movement
"random" "heuristic" "reinforcement-learning"
2

CHOOSER
28
169
222
214
robot-position
robot-position
"center" "random"
1

SWITCH
87
266
217
299
show-path
show-path
0
1
-1000

OUTPUT
137
12
575
63
12

SWITCH
87
303
244
336
show-node-values
show-node-values
0
1
-1000

@#$#@#$#@
# Christopher Pileggi- Project 2

### Description

Random movement - the robot chooses a direction to move at random. It has nothing to guide its movement, so it randomly decides to move either up, down, left or right from its current position. It continues to do this for each node until it finds the goal.

Heuristic movement - the robot moves in a direction according to a heuristic value instead of at random. From its current position, the robot looks at the heuristic values of its surrounding nodes. The node with the best heuristic value is the one that the robot moves to. It continues to do this until it reaches the goal. The heuristic that I used: h(n) = the shorter the distance between the goal node and a specific node, the greater the value that specific node has.

Reinforcement learning - every non-obstacle node in the world is evaluated and is assigned a value according to the Bellman Equation. Every node starts off with a value of 0. Each node is then assigned a value, which will be is the lowest value of its child nodes. Added to that value is the path cost between the node being evaluated and the child node with the lowest value. Every node in the world is evaluated this way. If the goal is evaluated, it is always assigned 0. If a node is blocked off from the goal or is an obstacle, it is given a value of -1. Once every node is evaluated, they are all reevaluated until the values of each node does not change from its previous evaluation. Once the values are stable, the robot looks at its surrounding nodes and moves to the node with the highest heuristic value. The robot continues this until it reaches the goal. In this case: h(n)=the smaller the node's value, the better

Extra features:
1) There is an output box included on the screen. When reinforcement learning is selected, it displays the final results of the value iteration in the box. If the goal is cut off from the robot, it prints failure. If the goal is not blocked, it will display the time it took (in seconds) to complete the value iteration.

2) For the reinforcement learning, the nodes are colored according to their heuristic values. The better the value, the lighter green it is. The worst the value is, the darker green it is. After a successful value iteration, it results in a color gradient of green. If a node is blocked from the robots path, it will be set to yellow.

3) There are two switches added. One displays the path the robot takes when  doing a reinforcement learning or a heuristic movement. Another displays the values of each node evaluated.

Default World: 100 obstacles; the maximum x and y coordinates are 10, and the minimum x and y coordinates are -10; the robot is positioned at random

## Experiment
### Random Movement

Trial  1: 7296
Trial  2: 455
Trial  3: 4614
Trial  4: 7115
Trial  5: 804
Trial  6: 1258
Trial  7: 4360
Trial  8: 2309
Trial  9: 2279
Trial 10: 1079
Trial 11: 2605
Trial 12: 3902
Trial 13: 1341
Trial 14: 1482
Trial 15: 10,182
Trial 16: 1816
Trial 17: 3825
Trial 18: 569
Trial 19: 1209
Trial 20: 2181

Average = 3034.05 (about 3034 ticks)

Standard Deviation = +/- 2598.78 ticks

### Heuristic Movement

Trial  1: 8
Trial  2: 6
Trial  3: 8
Trial  4: 28
Trial  5: 7
Trial  6: 25
Trial  7: 8
Trial  8: 15
Trial  9: 25
Trial 10: 32
Trial 11: 10
Trial 12: 11
Trial 13: 9
Trial 14: 19
Trial 15: 9
Trial 16: 13
Trial 17: 15
Trial 18: 21
Trial 19: 9
Trial 20: 30

Average = 15.4 (about 15 ticks)

Standard Deviation = +/- 8.51 ticks

### Reinforcement Learning

Trial  1: 7
Trial  2: 20
Trial  3: 9
Trial  4: 28
Trial  5: 7
Trial  6: 14
Trial  7: 12
Trial  8: 30
Trial  9: 4
Trial 10: 24
Trial 11: 21
Trial 12: 5
Trial 13: 8
Trial 14: 31
Trial 15: 23
Trial 16: 6
Trial 17: 13
Trial 18: 20
Trial 19: 29
Trial 20: 30

Average = 17.1 (about 17 ticks)

Standard Deviation = +/- 9.59 ticks

## Evaluation

The random movement has highest average and the highest standard deviation of the three. This makes sense because the robot moves at random, so the number of ticks the robot makes before it reaches the goal can range from really large or really small.

The heuristic movement has a low mean and low standard deviation. This is because the robot tends to move in the direction of the goal, due to evaluating the heuristic values of the nodes. The advantage of this method is that the robot moves very quickly to the goal. The disadvantage, however, is that the robot tends to hit dead ends if there is a set of obstacles blocking the robots path. When this is the case, the robot will become stuck and will never reach the goal during its run.

The reinforcement learning method has a low mean and low standard deviaton, similar to the heuristic movement. However, unlike the heuristic movement, the robot will always find the goal with this method unless the goal is blocked. This method also gives the robot the shortest path to the goal from its starting position. The only disadvantage of this method is that it takes time for the robot to learn the appropriate heuristic values of all the nodes.
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

robot
true
0
Circle -7500403 false true 2 2 297
Line -7500403 true 150 0 150 105

robot2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Rectangle -7500403 true true 150 30 150 135
Rectangle -7500403 true true 135 30 160 138

sheep
false
0
Rectangle -7500403 true true 151 225 180 285
Rectangle -7500403 true true 47 225 75 285
Rectangle -7500403 true true 15 75 210 225
Circle -7500403 true true 135 75 150
Circle -16777216 true false 165 76 116

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
NetLogo 5.3.1
@#$#@#$#@
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
