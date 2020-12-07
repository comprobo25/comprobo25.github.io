---
title: "Kalman Filter and Studio"
toc_sticky: true
---

## Today

* Kalman Filter
* Studio Time!

## For Next Time

* Work on your final projects!
* Readings for next Robots and Society discussion

## Kalman Filter

We'll be running through this slide deck (pptx, PDF)

Once we've given you the basic ideas, we'll also show you how to run your own simple Kalman filter.  The commands to run the filter are.


First, startup roscore.

```bash
$ roscore
```

In a new terminal, run the following command.
 
```bash
$ rosrun simple_filter simple_kalman.py
```

In a new terminal, run the following command.
```bash
rosrun rqt_gui rqt_gui
```

In the ``rqt_gui`` window, go to ``plugins``, ``Visualization``, ``Dynamic Reconfigure``.  You should now have a window that looks like this.

![The dynamic reconfigure GUI of the simple Kalman node](day24images/dynamicreconfigure.png)

You can use these sliders to control the behavior of the filter.
