cc.Class({
    extends: cc.Component,

    properties: {
        debug:true,
        gravity:cc.v2(0,-20)
        // foo: {
        //    default: null,      // The default value will be used only when the component attaching
        //                           to a node for the first time
        //    url: cc.Texture2D,  // optional, default is typeof default
        //    serializable: true, // optional, default is true
        //    visible: true,      // optional, default is true
        //    displayName: 'Foo', // optional
        //    readonly: false,    // optional, default is false
        // },
        // ...
    },

    // use this for initialization
    onLoad: function () {
        let physicsManager = cc.director.getPhysicsManager();
        physicsManager.enabled = true;
        physicsManager._world.SetGravity(this.gravity);
        if(this.debug){
            physicsManager.debugDrawFlags = 
                // 0;
                // cc.PhysicsManager.DrawBits.e_aabbBit |
                // cc.PhysicsManager.DrawBits.e_pairBit |
                // cc.PhysicsManager.DrawBits.e_centerOfMassBit |
                cc.PhysicsManager.DrawBits.e_jointBit |
                cc.PhysicsManager.DrawBits.e_shapeBit
            ;
        }
    },

    // called every frame, uncomment this function to activate update callback
    // update: function (dt) {

    // },
});