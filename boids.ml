(*
Code I typed in from Jon Harrop's "Ocaml Journal".
A couple of typos fixed and I found I had to add the init line.


compile with:

ocamlfind ocamlc -g -package lablgtk2 -linkpkg unix.cma boids.ml -o boids

*)


type vec2 = { x: float; y: float };;

let ( +| ) a b = { x = a.x +. b.x; y = a.y +. b.y };;
let ( -| ) a b = { x = a.x -. b.x; y = a.y -. b.y };;
let ( *| ) s b = { x = s *. b.x; y = s *. b.y };;

let dot a b = a.x *. b.x +. a.y *. b.y;;

let length a = sqrt(dot a a);;

(*
The simulation is controlled by four variables and a target that the boids aim towards. 
The four variables are exposed as GTK+ widgets in the GUI for the user to control and are used by the simulation code. 
We encapsulate the mutable current value and constant range of each variable in the following scale record type: *)


type scale = { lower: float; mutable value: float; upper: float };;

(* The population, cohesion, separation and alignment variables may then be defined as: *)

let population = { lower = 1.0; value = 100.; upper = 100. };;


let cohesion = { lower = 0.0; value = 10.; upper = 10. };;

let separation = { lower = 0.0; value = 1000.; upper = 10000. };;

let alignment = { lower = 0.0; value = 1000.; upper = 1000. };;

(* The boid's target is simply a mutable reference to a 2D vector: *)

let target = ref {x=512.; y=512.};;

(* The state of the simulation is composed of the current position and velocity of each boid, both of 
which are stored as vec2 array data structures called p and v , respectively: *)

let p =
    Array.init (int_of_float population.upper)
      (fun _ -> { x = Random.float 1000.; y = Random.float 1000. });;

let v = Array.create (int_of_float population.upper) {x=0.; y=0.};;

(* The following dt value and timer function are used to measure the elapsed real time since the last iteration of 
the simulation (the last invocation of the timer function): *)

let dt = ref 0.0;;

let timer =
    let t = ref(Unix.gettimeofday()) in
    fun () ->
        let t' = Unix.gettimeofday() in
        dt := t' -. !t;
        t := t';;

(* The following update function is responsible for simulating the dynamics of the boids: *)

let update() =
    let dt = !dt in
    for i=0 to int_of_float population.value - 1 do
      let p0 = p.(i) and v0 = v.(i) in
      let p0 = p0 +| dt *| v0 in
      let home = !target -| p0 in
      let v0 = v0 +| 1. /. (1. +. length home) *| home in
      p.(i) <- p0 +| dt *| v0;
      v.(i) <- v0;
      for j=0 to i-1 do
        let p0 = p.(i) and v0 = v.(i) in
        let p1 = p.(j) and v1 = v.(j) in
        let dp = p1 -| p0 in
        let sep = 1. +. length dp in
        let u v = 1. /. length v *| v in
        let dv =
          cohesion.value /. sep ** 2.0 *| dp +|
            alignment.value /. sep ** 3.0 *| (u v1 -| u v0) -|
            separation.value /. sep ** 4.0 *| dp in
        let dampen v =
          let speed = max 10. (min 100. (length v)) in
          speed /. length v *| v in
        p.(i) <- p0;
        v.(i) <- dampen(v0 +| dt *| dv);
        p.(j) <- p1;
        v.(j) <- dampen(v1 -| dt *| dv)
      done
    done;;

(*
This function begins by calculating the time elapsed since it was last invoked and then loops over each boid updating its dynamics. 
In this case, we have adopted the simplest possible O(n 2 ) algorithm that considers every boid in the context of every other boid.
Note that double counting is avoided by looping the inner loop over 0..i-1.
The outer loop begins by updating the position of each boid according to its velocity and adding the attraction to the target 
(that will be the mouse coordinate).

The inner loop obtains the positions and velocities for both the i th and j th boids and then calculates the displacement 
dp between them and the separation sep . The separation is incremented in order to prevent subsequent calculations from blowing 
up when a pair of boids are very close. A nested function u is defined that returns a normalized vector. 
The acceleration dv is the sum of components due to cohesion, alignment and separation. 
The cohesion is a simple inverse square law that attracts boids to each other. The alignment adjust the current velocity 
vector to direct it toward that of the neighbor, using the normalized velocity vectors, and the effect decays with the cube of 
the separation. Finally, the separation force is similar to the cohesive force but with the opposite sign and 
using a fourth power. So the separation force falls off very quickly as the distance increases but is much larger for small 
separations, achieving a kind of hard core repulsion.

The following expose callback is used to render the boids into a GTK+ drawable using a line for each boid where the length 
of the line is proportional to the boid's velocity: *)

let rec expose (g: GDraw.drawable) =
    let width, height = g#size in
    g#set_foreground `BLACK;
    g#rectangle ~filled:true ~x:0 ~y:0 ~width ~height ();
    g#set_foreground `WHITE;
    for i=0 to int_of_float population.value - 1 do
      let p = p.(i) and v = v.(i) in
      let p2 = p +| 3. *. !dt *| v in
      let x0 = int_of_float p.x and y0 = int_of_float p.y in
      let x1 = int_of_float p2.x and y1 = int_of_float p2.y in
      g#line x0 y0 x1 y1;
    done;
    g#set_clip_rectangle (Gdk.Rectangle.create 0 0 width height);
    false;;

(* The GUI is composed of the real-time boid visualization and a pane of sliders that allow the user to change each of the
 four variables. The GUI code required to handle each slider is very similar and may be productively factored out.

We begin with a function mk_label that creates a label with the given text and packs it using the given packing function: *)

let mk_label packing text =
    ignore(GMisc.label ~text ~packing ());;

(* The following mk_scale function uses a scale value of the scale type to construct a slider with the appropriate 
range and starting value as well as registering a callback function that updates the current value of the scale 
when the user alters it: *)

let mk_scale packing text scale =
    mk_label packing text;
    let adjustment =
      GData.adjustment
        ~lower:scale.lower
        ~value:scale.value
        ~upper:(scale.upper +. 1.0)
        ~page_size:1.0 () in
    let widget =
      GRange.scale
        `HORIZONTAL
        ~adjustment
        ~digits:0
        ~draw_value:true
        ~value_pos:`BOTTOM
        ~packing () in
    let callback() = scale.value <- widget#adjustment#value in
    ignore(widget#connect#value_changed ~callback);;

(* The following mk_area function creates a GTK+ drawable where the boids can be visualized: *)

let mk_area (box: GPack.box) =
    let area = GMisc.drawing_area ~packing:(box#pack ~expand:true) () in
    area#misc#realize ();
    let drawing = new GDraw.drawable area#misc#window in
    ignore(area#event#connect#expose ~callback:(fun _ -> expose drawing));
  
    let press (e: GdkEvent.Button.t) =
      target := { x = GdkEvent.Button.x e; y = GdkEvent.Button.y e };
      false in
    ignore(area#event#connect#button_press ~callback:press);
    area#event#add [`BUTTON_PRESS];
    area;;

(* This function also registers a callback that sets the target for the boids to the location of the
 mouse when it is clicked. Note the use of the area#event#add function to activate the handling of 
mouse button presses. Without this activation the event will 
never be fired and our callback will never be invoked.
The following mk_controls function builds the GUI controls that allow the user to update
 the variables of the simulation using sliders: *)

let mk_controls (box: GPack.box) =
    let controls = GPack.vbox ~packing:(box#pack ~padding:11) () in
    let packing = controls#pack ~padding:8 in
    mk_label packing "Controls";
    mk_scale packing "Population" population;
    mk_scale packing "Cohesion" cohesion;
    mk_scale packing "Separation" separation;
    mk_scale packing "Alignment" alignment;;

(* In order to provide good response times under heavy load without allowing the event queue 
to grow we shall be managing the GTK+ main event loop ourselves. In order to handle quitting 
of the program correctly, we shall use the following mutable to record if the user has closed the program: *)


let fin = ref false;;

(* Finally, the main program creates a GTK+ window, registers a callback to close the application 
if the user chooses to do so, creates a hbox containing the GTK+ drawable used for visualization 
and the GUI controls, displays the window and enters a custom main loop: *)


let () =
   let locale = GtkMain.Main.init () in

    let window =
      GWindow.window ~width:640 ~height:480 ~border_width:10 () in
    ignore(window#connect#destroy
            ~callback:(fun () -> fin := true; GMain.Main.quit()));
  
    let main = GPack.hbox ~packing:window#add () in
  
    let area = mk_area main in
    mk_controls main;
  
    window#show ();
  
    while not !fin do
      while Glib.Main.pending() do
        ignore(Glib.Main.iteration false);
      done;
      timer();
      update();
      GtkBase.Widget.queue_draw area#as_widget
    done;;
