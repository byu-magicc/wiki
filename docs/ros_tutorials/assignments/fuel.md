# Assignment 7: Display/Publish Fuel Remaining

Now create a subscriber node which subscribes to the `turtle1/pose` topic and uses the information provided over this topic to publish remaining fuel.  Test your node thoroughly.

## Example
```
[ INFO] [1441729129.808192832]: Fuel Remaining: 878.002
[ INFO] [1441729129.824126911]: Fuel Remaining: 877.502
[ INFO] [1441729129.840222084]: Fuel Remaining: 877.002
[ INFO] [1441729129.856043631]: Fuel Remaining: 876.502
[ INFO] [1441729129.872168505]: Fuel Remaining: 876.002
[ INFO] [1441729129.888102265]: Fuel Remaining: 875.502
[ INFO] [1441729129.904134376]: Fuel Remaining: 875.002
[ INFO] [1441729129.920143881]: Fuel Remaining: 874.502
```

Hint: A good example node within the `relative_nav` library is the `joy_commander` node located within the `joy_velocity` package.